#
#  Copyright (c) 2023 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#
#

from composer.model.stack import Stack
from composer.introspection.launcher import Ros2LaunchParent

UNKNOWN = 'unknown'
ACTIVE = 'active'
KILLED = 'killed'

class EdgeDevice:
    def __init__(self, twin):
        self.twin = twin
        self.definition = None
        self.state = UNKNOWN  
        self.current_stack = None
        self.launcher = Ros2LaunchParent()

    def _update_current_stack(self, definition=None, state=UNKNOWN):
        """Helper method to update the current stack based on the provided definition and state."""
        if definition is not None:
            self.current_stack = Stack(self, definition, None)
        self.state = state
        self.twin.set_current_stack(self.current_stack, state=self.state)

    def _handle_exception(self, action, exception):
        """Centralized exception handling."""
        print(f'An exception occurred during {action}: {exception}')

    def bootstrap(self):
        try:
            print("Edge Device bootstrapping...")
            current_definition = self.twin.get_current_stack().get('current', {})
            stack_id = current_definition.get('stackId')

            if stack_id:
                self.definition = self.twin.stack(stack_id)
                self.state = current_definition.get('state', UNKNOWN)
                self._update_current_stack(self.definition, self.state)
            else:
                self.state = UNKNOWN

            print('Edge Device bootstrap done.')
        except Exception as e:
            self._handle_exception('bootstrapping', e)

    def activate(self, current=None):
        try:
            if self.current_stack:
                self.current_stack.kill_all(self.launcher)
            self._update_current_stack(current, ACTIVE)
            self.current_stack.launch(self.launcher)
        except Exception as e:
            self._handle_exception('activation', e)

    def apply(self, current=None):
        try:
            if self.current_stack:
                self.current_stack.kill_diff(self.launcher, self.current_stack)
            self._update_current_stack(current, ACTIVE)
            self.current_stack.apply(self.launcher)
        except Exception as e:
            self._handle_exception('applying changes', e)

    def kill(self, payload=None):
        try:
            if self.current_stack:
                self.current_stack.kill_all(self.launcher)
            self._update_current_stack(payload, KILLED)
        except Exception as e:
            self._handle_exception('killing process', e)

    def stack(self, stackId):
        """Retrieve stack definition by ID."""
        return self.twin.stack(stackId)
