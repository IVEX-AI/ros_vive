from threading import Thread, Event, Lock


class AsyncExclusiveActionExecutor(object):

    def __init__(self):
        # When is_shutdown is true, no more actions should be executed
        self.is_shutdown = False

        # Only one action can be active at the same time.
        # Actions are executed in a separate thread since they may be blocking
        # When a new action comes in, its thread is as set as the next_thread,
        # which will be executed after the current_thread finishes
        self.current_thread = None
        self.next_thread = None

        # process_event is a synchronized data type which is used to message to threads that they need to stop
        # This can be during shutdown, or when another action is queued
        self.process_event = Event()

        # The manage function runs asynchronously in a separate thread as a daemon
        self.management_thread = Thread(target=self.manage)
        self.management_thread.daemon = True

        # The queue_action function runs in the main thread,
        # so this lock prevent synchronization issues when queue_action is called during a manage execution loop
        self.management_lock = Lock()

        self.management_thread.start()

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        with self.management_lock:
            self.is_shutdown = True
            self.process_event.set()

    def manage(self):
        while True:

            # We only need to manage things if an action is going to stop
            self.process_event.wait()

            # If an action is currently active, wait until it completes
            if self.current_thread is not None:
                self.current_thread.join()

            if self.is_shutdown:
                break

            # If we got here, the currently is no active action
            with self.management_lock:

                # Clear the process_event flag to allow the next action to run
                self.process_event.clear()

                # If there is a next action to execute, do so
                if self.next_thread is not None:
                    self.current_thread = self.next_thread
                    self.next_thread = None
                    self.current_thread.start()

    # The function pass to func should accept 1 argument more than is passed to args.
    # The final argument will be the process_event flag
    def queue_action(self, func, args):
        with self.management_lock:
            # Set the process_event flag to indicate to any currently active action that it needs to stop
            self.process_event.set()
            self.next_thread = Thread(target=func, args=[x for x in args] + [self.process_event])
            self.next_thread.daemon = True
