
class RuntimeException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, 'Runtime Exception', *args)


class SetupException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, 'Setup Exception', *args)


class TimingViolationException(RuntimeException):
    def __init__(self, *args):
        RuntimeException.__init__(self, 'Timing violation', *args)


class FreshnessConstraintViolationException(TimingViolationException):
    def __init__(self, *args):
        TimingViolationException.__init__(
            self, 'Freshness constriant violation', *args)


class DataAbsenceException(RuntimeException):
    def __init__(self, *args):
        RuntimeException.__init__(self, 'Data absence', *args)


class DataCorruptionException(RuntimeException):
    def __init__(self, *args):
        RuntimeException.__init__(self, 'Data corruption', *args)


class InvalidStreamDataException(DataCorruptionException):
    def __init__(self, *args):
        DataCorruptionException.__init__(self, 'Invalid stream data', *args)


class InvalidEventException(DataCorruptionException):
    def __init__(self, *args):
        DataCorruptionException.__init__(self, 'Invalid event', *args)


class InvalidModeChangeException(DataCorruptionException):
    def __init__(self, *args):
        DataCorruptionException.__init__(self, 'Invalid mode change', *args)

class ModeManagerAbsenceException(RuntimeException):
    def __init__(self, *args):
        RuntimeException.__init__(self, 'Mode manager absence', *args)

class InvalidUsageException(SetupException):
    def __init__(self, *args):
        SetupException.__init__(self, 'Invalid usage', *args)
