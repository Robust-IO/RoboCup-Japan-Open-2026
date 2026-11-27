"""Non-blocking line commands for a foreground supervisor; EOF is not exit."""
import select
class ConsoleExit:
    def __init__(self,stream):
        self.stream=stream;self.enabled=True
        try:self.fd=stream.fileno()
        except (ValueError,OSError,AttributeError):self.enabled=False
    def poll(self):
        if not self.enabled:return False
        try:
            if not select.select([self.fd],[],[],0)[0]:return False
            line=self.stream.readline()
        except (ValueError,OSError):self.enabled=False;return False
        if line=='':self.enabled=False;return False
        return line.strip().casefold() in ('q','quit','exit','退出')
