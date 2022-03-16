
class PeriodicDebugger:
    def __init__(self, publisher, period=100):
        self.publisher = publisher
        self.period = period
        self.debuggers = {'info': 0}

    def register(self, name):
        self.debuggers[name] = 0

    def debug(self, msg, debugger='info'):
        if debugger not in self.debuggers:
            raise Exception("You have not register this debugger.")

        if self.debuggers[debugger] > self.period:
            self.publisher.publish("[" + debugger + "]: " + msg)
            self.debuggers[debugger] = 0
        else:
            self.debuggers[debugger] += 1

