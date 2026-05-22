from abc import ABC, abstractmethod

class DroneInterface(ABC):
    
    @abstractmethod
    def get_position(self):
        pass

    @abstractmethod
    def get_ranges(self):
        pass

    @abstractmethod
    def set_velocity(self, velocity):
        pass

    @abstractmethod
    def takeoff(self):
        pass

    @abstractmethod
    def land(self):
        pass
    
    @abstractmethod
    def emergency_stop(self):
        pass

    @abstractmethod
    def is_connected(self):
        pass
