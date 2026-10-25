"""Single Arduino transaction; no ROS or actuator access."""
import json
import math
import uuid


class ChangerTransaction:
    def __init__(self, command, timeout, now):
        if command not in ('LOCK', 'UNLOCK'):
            raise ValueError('tool changer command must be LOCK or UNLOCK')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('tool changer timeout must be positive and finite')
        self.id = uuid.uuid4().hex
        self.command = command
        self.deadline = now + timeout
        self.complete = False
        self.error = ''
        self.stopped = False

    def request(self):
        return json.dumps(dict(id=self.id, command=self.command))

    def check(self, now):
        if not self.complete and not self.error and now >= self.deadline:
            self.error = 'Arduino completion timeout'
        if self.error:
            raise RuntimeError(self.error)
        return self.complete

    def receive(self, text, now):
        if self.complete or self.error:
            return
        self.check(now)
        try:
            msg = json.loads(text)
            if not isinstance(msg, dict) or msg.get('id') != self.id:
                return
            state = msg.get('state')
            if state == 'ERROR':
                self.error = 'Arduino ERROR: '+str(msg.get('detail', ''))[:256]
            elif state == ('LOCKED' if self.command == 'LOCK' else 'UNLOCKED'):
                self.complete = True
        except (TypeError, ValueError):
            return

    def abort(self, reason):
        # Abort wins even if completion arrived just before cancellation.
        self.error = reason
        self.complete = False
        if not self.stopped:
            self.stopped = True
            return json.dumps(dict(id=self.id, command='STOP'))
        return None
