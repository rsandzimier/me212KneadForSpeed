#Python doubly linked list implementation

class Node():
	data = None
	prev_node = None
	next_node = None

	def __init__(self, data, p, n):
		self.data = data
		self.prev_node = p
		self.next_node = n

class LinkedList():
	
	head = None #start of list
	tail = None #end of list
	length = 0

	def __init__(self):
		#nothing to do
		pass

	def append(self, item): #adds item to end of linked list
		n = Node(item, self.tail, None)
		if (self.tail != None):
			self.tail.next_node = n
		else: #first item in list
			self.head = n
		self.tail = n
		self.length = self.length + 1

	def get_node(self, index):
		n = self.head
		i=0
		while (i<index):
			if (n == None):
				raise IndexError("Linked List index out of range: "+str(i))
			n = n.next_node
			i = i+1
		if (n == None):
			raise IndexError("Linked List index out of range: "+str(i))
		return n

	def pop(self, index):
		n = self.get_node(index)
		if (n != self.head):
			n.prev_node.next_node = n.next_node
		else:
			self.head = n.next_node #we popped the head node, so need to move reference
		if (n != self.tail):
			n.next_node.prev_node = n.prev_node
		else:
			self.tail = n.prev_node #we popped the tail node, so need to move reference
		self.length = self.length - 1
		return n.data

	def insert(self, index, item):
		n_0 = self.get_node(index)
		n = Node(item, n_0.prev_node, n_0)
		n_0.prev_node.next_node = n
		n_0.prev_node = n
		if (n_0 == self.head):
			self.head = n
		self.length = self.length + 1

	def get(self, index):
		return self.get_node(index).data

'''if __name__ == "__main__":
	l = LinkedList()
	l.append(1)
	#print(l.get(0))
	#print(l.get(1))
	l.append(4)
	#print(l.get(0))
	#print(l.get(1))
	l.append(67)
	l.append(53)
	l.insert(2, 100)
	print(l.get(2))
	print(l.get(3))
	print(l.pop(0))
	print(l.get(2))
	print(l.length)
	print(l.pop(0))
	print(l.pop(2))
	print(l.get(1))
	l.append(1000)
	print(l.get(l.length-1))'''


