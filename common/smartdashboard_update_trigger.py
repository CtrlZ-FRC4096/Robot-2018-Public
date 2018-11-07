"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2018
Code for robot "Atla-Z"
contact@team4096.org
"""

from wpilib.buttons import Trigger
from networktables import NetworkTables


class SmartDashboard_Update_Trigger(Trigger):
    '''
    Trigger used to check when entries are updated
    in the SmartDashboard.  Courtesy of Team 2423.
    '''

    def __init__(self, table_key, default_value):
        '''
        Creates a new trigger for entries when updated.

        :param table_key: The name of the entry in the
        SmartDashboard NetworkTable
        :param default_value: The value the entry will
        take if it doesn't already exist in the
        SmartDashboard
        '''

        self.table_key = table_key
        self.sd = NetworkTables.getTable('SmartDashboard')
        self.auto_update_value = self.sd.getAutoUpdateValue(table_key, default_value)
        self.last_value = self.auto_update_value.get()

    def get(self):
        updated = self.auto_update_value.get() != self.last_value
        self.last_value = self.auto_update_value.get()

        return updated

    def get_table_key(self):
        return self.table_key

    def get_key_value(self):
        return self.auto_update_value.get()