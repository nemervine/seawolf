
/* Local includes */
#include "seawolf.h"
#include "seawolf_hub.h"

/* Networking includes */
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>

/* Global client information */
static int client_socks[MAX_CLIENTS]; /* client socket descriptors */
static bool client_authenticated[MAX_CLIENTS]; /* client authentication status */
static short client_errors[MAX_CLIENTS]; /* successive errors from a client */

static Comm_Message* Hub_Net_receiveMessage(int comm_socket) {
    Comm_PackedMessage* packed_message;
    Comm_Message* message;
    uint16_t total_data_size;
    size_t n;

    if(recv(comm_socket, &total_data_size, sizeof(uint16_t), MSG_WAITALL|MSG_PEEK) != sizeof(uint16_t)) {
        return NULL;
    }

    total_data_size = ntohs(total_data_size);

    packed_message = Comm_PackedMessage_new();
    packed_message->length = total_data_size + COMM_MESSAGE_PREFIX_LEN;
    packed_message->data = malloc(packed_message->length);

    if((n = recv(comm_socket, packed_message->data, packed_message->length, MSG_WAITALL)) != packed_message->length) {
        Hub_Logging_log(WARNING, Util_format("Received invalid message: %d %d", packed_message->length, n));
        Comm_PackedMessage_destroy(packed_message);
        return NULL;
    }

    message = Comm_unpackMessage(packed_message);
    Comm_PackedMessage_destroy(packed_message);
    
    return message;
}

static bool Hub_Net_sendPackedMessage(int comm_socket, Comm_PackedMessage* packed_message) {
    static pthread_mutex_t send_lock = PTHREAD_MUTEX_INITIALIZER;
    int n;

    /* Send data */
    pthread_mutex_lock(&send_lock);
    n = send(comm_socket, packed_message->data, packed_message->length, 0);
    pthread_mutex_unlock(&send_lock);

    return (n != -1);
}

static bool Hub_Net_sendMessage(int comm_socket, Comm_Message* message) {
    Comm_PackedMessage* packed_message = Comm_packMessage(message);
    int n;

    /* Send packed message */
    n = Hub_Net_sendPackedMessage(comm_socket, packed_message);
    
    /* Destroy sent message */
    Comm_PackedMessage_destroy(packed_message);

    return (n != -1);
}

static void Hub_Net_removeClient(int i) {
    shutdown(client_socks[i], SHUT_RDWR);
    client_socks[i] = 0;
    client_authenticated[i] = false;
    client_errors[i] = 0;
}

void Hub_Net_mainLoop(void) {
    int svr_sock = 0; /* server socket */
    int client_new = 0; /* temp place for incoming connections */
    const int reuse = 1; /* passed to setsockopt to allow reuse of socket */
    fd_set fdset_mask_r; /* file descriptor set passed to select call */
    struct sockaddr_in svr_addr; /* Server binding address/port structure */
    Comm_Message* client_message;
    Comm_Message* response;
    Comm_PackedMessage* packed_response;
    int respond_to;
    int n, i, j; /* misc */

    /* Clear all descriptors */
    memset(client_socks, 0, sizeof(int) * MAX_CLIENTS);

    /* Set all authentication status to false */
    memset(client_authenticated, 0, sizeof(bool) * MAX_CLIENTS);

    /* Zero all error counts */
    memset(client_errors, 0, sizeof(short) * MAX_CLIENTS);

    /* Initialize the connection structure to bind the the correct port on all
       interfaces */
    svr_addr.sin_family = AF_INET;
    svr_addr.sin_addr.s_addr = 0; /* 0.0.0.0 - all interfaces */
    svr_addr.sin_port = htons(COMM_PORT);
    
    /* Create the socket */
    svr_sock = socket(AF_INET, SOCK_STREAM, 0);
    if(svr_sock == -1) {
        Hub_Logging_log(CRITICAL, Util_format("Error creating socket: %s", strerror(errno)));
        Hub_exitError();
    }

    /* Allow localhost address reuse. This allows us to restart the hub after it
       unexpectedly dies and leaves a stale socket */
    setsockopt(svr_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    /* Bind the socket to the server port/address */
    if(bind(svr_sock, (struct sockaddr*) &svr_addr, sizeof(svr_addr)) == -1) {
        Hub_Logging_log(CRITICAL, Util_format("Error binding socket: %s", strerror(errno)));
        Hub_exitError();
    }

    /* Start listening */
    listen(svr_sock, MAX_CLIENTS);
    
    /* Begin accepting connections */
    Hub_Logging_log(INFO, "Accepting client connections");
    
    /* Start sending/recieving messages */
    while(true) {
        /* Zero of the file descriptor set */
        FD_ZERO(&fdset_mask_r);

        /* Add the server socket to the set */
        FD_SET(svr_sock, &fdset_mask_r);

        /* Add each client to the set */
        for(i = 0; i < MAX_CLIENTS; i++) {
            if(client_socks[i] != 0) {
                FD_SET(client_socks[i], &fdset_mask_r);
            }
        }

        /* Perform the select call, return -1 is an error, and 0 means no
           results */
        n = select(FD_SETSIZE, &fdset_mask_r, NULL, NULL, NULL);
        if(n < 0) {
            Hub_Logging_log(ERROR, Util_format("Error selecting from descriptors, attempting to continue: %s", strerror(errno)));
            continue;
        } else if(n == 0) {
            /* Select returned no active sockets. Ignore it and try again */
            continue;
        }

        /* If the server socket is set then a new connection is coming
           in. Handle it */
        if(FD_ISSET(svr_sock, &fdset_mask_r)) {
            client_new = accept(svr_sock, NULL, 0);
        }

        /* Check for incoming data */
        for(i = 0; i < MAX_CLIENTS; i++) {
            if(client_socks[i] == 0 && client_new) {
                /* If client_socks[i] is 0 and client_new is non zero we have a
                   place to store the descriptor for the client connection we
                   just accepted */
                client_socks[i] = client_new;
                client_authenticated[i] = false;
                client_new = 0;
                Hub_Logging_log(INFO, "Accepted new client connection");
            } else if(client_socks[i] != 0 && FD_ISSET(client_socks[i], &fdset_mask_r)) {
                /* Read message from the client  */
                client_message = Hub_Net_receiveMessage(client_socks[i]);
                if(client_message == NULL) {
                    client_errors[i]++;
                    if(client_errors[i] > MAX_ERRORS) {
                        Hub_Logging_log(ERROR, "Excess read errors, dropping client");
                        Hub_Net_removeClient(i);
                    }
                    continue;
                }
                
                /* Process message */
                respond_to = Hub_Process_process(client_message, &response, &client_authenticated[i]);

                if(respond_to == RESPOND_TO_SENDER) {
                    n = Hub_Net_sendMessage(client_socks[i], response);
                    if(!n) {
                        /* Failed to send, shutdown client */
                        Hub_Logging_log(ERROR, "Client disconnected, shutting down client");
                        Hub_Net_removeClient(i);
                    }
                } else if(respond_to == RESPOND_TO_ALL) {
                    packed_response = Comm_packMessage(response);
                    for(j = 0; j < MAX_CLIENTS; j++) {
                        if(client_socks[j] && client_authenticated[j]) {
                            n = Hub_Net_sendPackedMessage(client_socks[j], packed_response);
                            if(!n) {
                                /* Failed to send, shutdown client */
                                Hub_Logging_log(ERROR, "Client disconnected, shutting down client");
                                Hub_Net_removeClient(j);
                            }
                        }
                    }
                    Comm_PackedMessage_destroy(packed_response);
                }

                if(response) {
                    for(j = 0; j < response->count; j++) {
                        free(response->components[j]);
                    }
                    Comm_Message_destroy(response);
                }
                Comm_Message_destroyUnpacked(client_message);
            }
        }

        /* We have a new client but we weren't able to store them anywhere. Max
           clients limit has been exceded */
        if(client_new) {
            Hub_Logging_log(ERROR, "Hub has excessive client count!");
        }
    }

    /* Shut down server socket */
    shutdown(svr_sock, SHUT_RDWR);
}