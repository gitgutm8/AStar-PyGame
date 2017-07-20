import heapq
import itertools


class PriorityQueue:
    """
    Implementation of a priority queue, using the heapq module.
    """

    REMOVED = '<removed-task>'

    def __init__(self, *init_tasks):
        """
        Initialises a `PriorityQueue` instance, optionally takes
        some initial tasks.

        :param init_tasks: Collection of task-priority pairs
        """
        self.heap = []
        self.counter = itertools.count()
        self.entry_finder = {}
        for init_task in init_tasks:
            self.add_task(*init_task)

    def add_task(self, task, priority):
        """Add a new task or update the priority of an existing task"""
        if task in self.entry_finder:
            self.remove_task(task)
        count = next(self.counter)
        entry = [priority, count, task]
        self.entry_finder[task] = entry
        heapq.heappush(self.heap, entry)

    def remove_task(self, task):
        """Mark an existing task as REMOVED.  Raise KeyError if not found."""
        entry = self.entry_finder.pop(task)
        entry[-1] = PriorityQueue.REMOVED

    def pop_task(self):
        """Remove and return the lowest priority task. Raise KeyError if empty."""
        while self.heap:
            priority, count, task = heapq.heappop(self.heap)
            if task is not PriorityQueue.REMOVED:
                del self.entry_finder[task]
                return task
        raise KeyError('pop from an empty priority queue')

    def __bool__(self):
        return bool(self.heap)
