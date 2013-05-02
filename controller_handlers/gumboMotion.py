"""LTLMoP motion handler for the JR platform."""

from motion import MotionController
from pose import poseHandler

class motionControlHandler(object):
    """Send drive commands using MotionController."""

    def __init__(self, proj, shared_data, init_node=False):
        self._name = type(self).__name__
        self._controller = MotionController(init_node)
        self._next_region = None
        self._regions = proj.rfi.regions
        self._handlers = proj.h_instance

    def gotoRegion(self, current_region, next_region):
        """Try to drive to next_region, return whether we have arrived."""
        # Look up regions
        # TODO: Add error checking
        current_region = self._regions[current_region].name
        next_region = self._regions[next_region].name

        if current_region == next_region:
            # Reset next_region, and report that we are there already.
            self._next_region = None
            return True
        elif self._next_region == next_region:
            # We're already trying to go there. Check whether we've arrived.
            if self._at_destination():
                self._next_region = None
                print "{}: Arrived at destination {!r}.".format(
                    self._name, next_region)
                return True
            else:
                return False
        else:
            # Double check that we're not already there.
            if self._at_destination():
                # This should only occur in the rare case that the
                # robot has changed regions since FSA last checked.
                print "{}: Already at destination {!r}.".format(
                    self._name, next_region)
                return True
            else:
                print "{}: Moving from {!r} to {!r}.".format(
                    self._name, current_region, next_region)
                self._next_region = next_region
                found_region = self._controller.send_move_command(next_region)
                if not found_region:
                    print "{}: Cannot find region {!r}.".format(
                    self._name, next_region)
                return False

    def _at_destination(self):
        """Return whether we have reached our destination."""
        return self._get_location() == self._next_region

    def _get_location(self):
        """Return the current region."""
        return self._handlers['pose'].get_location()
