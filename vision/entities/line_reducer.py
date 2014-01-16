# line_reducer.py
# Written by Matthew Zalesak for NCSU Underwater Robotics
# November 14, 2013


from __future__ import division # '/' returns float instead of int.
import cv2
import numpy
import math

__version__ = '1.0.1'
		

class LineReducer(object):
	'''A class that reduces a large set of line segments into a smaller set of
	fundamental line segments.  The class may not function properly if there
	are fewer than 5 points.
	
	
	This class uses the following algorithm:
	
		First, define an error ratio.  Since we know that the lines we have
	images of might have some width, we set the error ratio to be the maximum
	acceptable value of the width of the line over the length.
		We also define tolerance and reject line ratio, coeficients used to
	determine whether lines are members of a cluster.
	
	Now, while we still have ungrouped line segments, do:
	1. Choose the longest line available and add it to a new group.
	2. Starting with the longest available lines, test if adding that line to
	the group makes the group violate the error ratio.  We assume that if
	adding the line violates the error ratio, then the line segment must belong
	to another line.  Do not test a line that is too far away from the long
	line we started with, as calculated using tolerance and recject line ratio.
	3. Continue.
	
	Now that we have many groups, we will discard those that are not really
	groups.  There is currently no algorithm to determine how to do this.
	
	Finally, take each group and do the following:
	1. Create a list of points made from the endpoints of each of the line
	segments in the group.
	2. Use the cv2.minAreaRect(points) function to find the rectangle of best
	fit for the points.
	3. Using the results of part 2, convert the rectangles into line form.
	4. Continue.
	
	Now return the list of lines that was produced.	
	'''
	_error_ratio = 1/5
	_reject_ratio = 1/3
	_tolerance = 1/5
	_list_lines = []
	_list_lines_count = 0
	_list_grouped = []
	_groups = []
	_final_lines = []
	
	
	def __init__(self, lines, error_ratio=1/5, reject_ratio=1/3, tolerance=1/5):
		'''Create a new instance of the LineReducer class.  All processing
		happens during initialization.
		
		Keyword Arguments:
		lines -- tuple of lines.  If lines contains only one line, this class
			may throw a TypeError exception.
		error_ratio -- float representing maximum width to height ratio of a
			box fitting around all lines segments that describe the same real
			line.
		reject_ratio -- Defines how far at least one point on a line segment
			can be away from the line in a side-to-side sense and still be
			conidered to be tested to be part of that line, where the distance
			is the main line's length multiplied by this ratio.
		tolerance -- Defines what percentage further past the ends of the line
			a new segment can be and still be considered part of that line.
		'''
		assert error_ratio < 1, "Error ratio must be less than one."
		assert error_ratio > 0, "Error ratio must be greater than zero."
		
		self._error_ratio = error_ratio
		self._reject_ratio = reject_ratio
		self._tolerance = tolerance
		
		self._list_lines_count = len(lines)
		self._list_lines = list(lines)
		
		
	def _line_length(self, line):
		'''Find the length of a line only given the tuple representing the
		endpoints of the line.
		
		Keyword Arguments:
		line -- tuple representing a line.
		'''
		square = (line[1][0] - line[0][0]) ** 2 + (line[1][1] - line[0][1]) ** 2
		return square ** (1/2)
		
		
	def _sort_list_lines(self, line_list):
		'''Sort all of the lines in line_list in order of longest to shortest.'''
		local_list = []
		for l in line_list:
			local_list.append([l, self._line_length(l)])
			
		sorted_list = sorted (local_list, key = lambda p : p[1] , reverse=True)
		
		final_list = []
		for l in sorted_list:
			final_list.append(l[0])	
				
		return final_list
	
	
	def _index_longest_available(self):
		'''Determine the index of the longest line segment that is not grouped.'''
		if self._list_lines_count == 0:
			return -1
		for x in xrange(self._list_lines_count):
			if not self._list_grouped[x]:
				return x
		return -1
		
		
	def _fit_rectangle(self, lines):
		'''Take a list of lines and find a rectangle that best matches them,
		such that all lines segments are entirely within the rectangle.
		
		1. First we gather all of the endpoints into a cloud.
		2. Then we use the cv2.fitEllise function to find the center and angle
		of best fitting	line through the cloud such that it includes all points.
		
		Note about format of rectangle:
		((a, b), (c, d), e)
		a -- x-coordinate of center of rectangle.
		b -- y-coordinate of center of rectangle.
		c -- width of rectangle
		d -- height of rectangle
		e -- angle in degrees, zero when width is completely horizontal.
		
		Keyword Arguments:
		lines -- list of tuples representing lines.
		'''
		cloud = []
		for line in lines:
			cloud.append(line[0])
			cloud.append(line[1])
		
		return cv2.minAreaRect(numpy.array(cloud))
		
		
	def _test_fit(self, lines):
		'''Take a group of line segments and determine if they violate the
		error ratio.
		
		Keyword Arguments:
		lines -- list of tuples representing line segments.
		Returns True if the segments are acceptable.
		'''
		rectangle = self._fit_rectangle(lines)
		
		try:
			ratio = rectangle[1][0] / rectangle[1][1]
		except ZeroDivisionError:
			return True		
		if ratio > 1:
			ratio = 1 / ratio
			
		if ratio > self._error_ratio:
			return False
		return True
		
		
	def _dist_points(self, point_a, point_b):
		'''Determine the distance between two points that are represented as
		tuples.
		'''
		square = (point_b[0] - point_a[0]) ** 2 + (point_b[1] - point_a[1]) ** 2
		return square ** (1/2)
		
	
	def _dot_prod(self, line_a, line_b):
		'''Find the dot product of two lines.
	
		Keyword Arguments:
		line_a -- tuple line segment representing a vector.
		line_b -- tuple line segment representing a vector.
		Returns number representing the dot product of the two vectors.'''
		a_delta_x = line_a[1][0] - line_a[0][0]
		b_delta_x = line_b[1][0] - line_b[0][0]
		a_delta_y = line_a[1][1] - line_a[0][1]
		b_delta_y = line_b[1][1] - line_b[0][1]
		return (a_delta_x * b_delta_x) + (a_delta_y * b_delta_y)	
		
	
	def _rejection_line(self, point, line):
		'''Find the rejection vector of two lines and return the resulting line.
		The math used here is well explained at:
		http://en.wikipedia.org/wiki/Vector_projection#Vector_rejection
	
		Keyword Arguments:
		point -- tuple representin point
		line -- tuple representing line
		'''
		a = (line[0], point)
		scalar = self._dot_prod(a, line)/(self._line_length(line) ** 2)
		delta_x = line[1][0] - line[0][0]
		delta_y = line[1][1] - line[0][1]
		
		line_endpoint = (line[0][0] + scalar * delta_x,\
							line[0][1] + scalar * delta_y)
		return (line_endpoint, point)
		
		
	def _min_dist_rejection(self, line_a, line_b):
		'''Determinie if the minimum distance between two lines is less than or
		equal to a specified ratio, measured using a rejection line.
		
		Keywork Arguments:
		line_a -- index of first line, line that determines min acceptable dist.
		line_b -- index of second line
		Returns True if they are close enough.
		'''
		l1 = self._list_lines[line_a]
		l2 = self._list_lines[line_b]
		
		rejection1 = self._rejection_line(l2[0], l1)
		rejection2 = self._rejection_line(l2[1], l1)
		
		length1 = self._line_length(rejection1)
		length2 = self._line_length(rejection2)
		
		comparison_length = self._line_length(self._list_lines[line_a])
		
		if length1 <= (comparison_length * self._reject_ratio)\
			or length2 <= (comparison_length * self._reject_ratio):
			return True
		return False
		
		
	def _min_dist_concentric(self, line_a, line_b):
		'''Determinie if the minimum distance between two lines is less than or
		equal to a specified ratio, measured by making sure at least one point
		on the line is within 100(1 + tolerance)% away from both endpoint of
		the main line.
		
		Keywork Arguments:
		line_a -- index of first line, line that determines min acceptable dist.
		line_b -- index of second line
		Returns True if they are close enough.
		'''
		l1 = self._list_lines[line_a]
		l2 = self._list_lines[line_b]
		
		allowable_dist = self._line_length(l1) * (1 + self._tolerance)
		
		if self._dist_points(l1[0], l2[0]) > allowable_dist\
			and self._dist_points(l1[0], l2[1]) > allowable_dist:
			return False
			
		if self._dist_points(l1[1], l2[0]) > allowable_dist\
			and self._dist_points(l1[1], l2[1]) > allowable_dist:
			return False
		return True		
		
		
	def _form_group(self):
		'''Forms a group starting with the longest available line and then adds
		it to the list self._group.
		'''
		this_group = []
		
		start_line = self._index_longest_available()
		if start_line == -1:  # -1 means no groups available.
			return False
		this_group.append(start_line)
		self._list_grouped[start_line] = True
		
		# Now we start adding lines.  Reasons that we might reject a line:
		#
		# 1. If the index is out of range.
		# 2. If it has already been added to another group.
		# 3. If it is more than half of the length of the first line away.
		# 4. If adding it makes the group of lines violate the error ratio.
		
		index = start_line
		while True:	
			index += 1
			if index >= self._list_lines_count:
				break
			if self._list_grouped[index]:
				continue
			if not self._min_dist_rejection(start_line, index)\
				or not self._min_dist_concentric(start_line, index):
				continue
				
			test_group = []
			for l in this_group:
				test_group.append(self._list_lines[l])
			test_group.append(self._list_lines[index])
			
			if not self._test_fit(test_group):
				continue
			
			this_group.append(index)
			self._list_grouped[index] = True
			
		# Finish up:
		self._groups.append(this_group)
		return True	

		
	def _group_to_line(self, group_index):
		'''Take the group at index 'group_index' and convert it into a best
		fitting line.
		
		Keyword Arguments:
		group_index -- index of group to process.
		Returns a tuple representing the line of best fit throught the group.
		'''
		# First we define the rectangle, then we use the angle to find the ends
		# of the line, we create the line, then we return.
		lines = []
		for l in self._groups[group_index]:
			lines.append(self._list_lines[l])		
		rectangle = self._fit_rectangle(lines)
		# Recall:
		# center_point = rectangle[0]
		# width = rectangle[1][0]
		# height = rectangle[1][1]
		# angle = rectangle[2]
		
		# First check if the rectangle accidentally mixed up width and height.	
		if rectangle[1][0] > rectangle[1][1]: # ... as it should be,
			length = rectangle[1][0]
			angle = rectangle[2]
		else:
			length = rectangle[1][1]
			angle = rectangle[2] + 90
		
		delta_x = length * math.cos(math.radians(angle)) / 2
		delta_y = length * math.sin(math.radians(angle)) / 2
		
		top_point = (rectangle[0][0] + delta_x, rectangle[0][1] + delta_y)
		bottom_point = (rectangle[0][0] - delta_x, rectangle[0][1] - delta_y)

		return (top_point, bottom_point)
		
	
	def _line_reduce(self):
		'''This is the actual function that executes the algorithm for the
		class.
		'''
		while True:
			result = self._form_group()
			if not result:
				break
				
		# self._DISCARD_USELESS_ONES()
		
		for g in xrange(len(self._groups)):
			self._final_lines.append(self._group_to_line(g))
			
			
	def calculate_lines(self):
		'''Returns a tuple containing all lines found.  Each line is
		represented by a tuple describing the end points.  Each point is
		represented by a tuple with x and y coordinates.
		
		The X and Y coordinates are FLOATS and may be POSITIVE or NEGATIVE.
		'''
		# Test for the weird case where someone enters one point and python
		# truncates the tuple in an incompatable way.
		# The error is this: If one point is entered, then the user thinks they
		# have entered (((a,b), (c,d))), but python may have changed it to
		# ((a,b), (c,d)), which causes errors when this class tries to read the
		# indecies of the tuple.
		if len(self._list_lines) == 2:
			try:
				self._list_lines[0][0][0]
			except TypeError:
				return self._list_lines
		
		# DO NOT REMOVE THESE TWO LINES EVER FOR ANY REASON OR ELSE.
		# If you do, you will likely crash the computer.
		self._groups = []
		self._final_lines = []
		
		
		self._list_grouped = [False for x in range(self._list_lines_count)]
		self._list_lines = self._sort_list_lines(self._list_lines)
		self._line_reduce()
		t = tuple(self._final_lines)
		return t
