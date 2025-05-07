#!/usr/bin/env python3  
from cereal import car  
  
# Button types from car.CarState.ButtonEvent.Type  
ButtonType = car.CarState.ButtonEvent.Type  
  
class ButtonSimulator:  
  def __init__(self):  
    self.reset()  
      
  def reset(self):  
    """Reset all button states."""  
    self.cancel = False  
    self.resume = False  
    self.set = False  
    self.accel = False  
    self.decel = False  
    self.gap_adjust = False  
      
  def simulate_button(self, CC, button_type, duration=1):  
    """  
    Simulate pressing a cruise control button.  
      
    Args:  
      CC: The car control object that contains cruiseControl  
      button_type: String representing the button to press  
      duration: Number of frames to keep the button pressed  
      
    Returns:  
      The modified CC object  
    """  
    # Reset all button states first  
    self.reset()  
      

    
    if button_type == "cancel":  
      CC.cruiseControl.cancel = True  
    elif button_type == "resume":  
      CC.cruiseControl.resume = True  
    elif button_type == "accel":  
      # This would need to be implemented in the car controller  
      CC.cruiseControl.accelCruise = True  
    elif button_type == "decel":  
      # This would need to be implemented in the car controller  
      CC.cruiseControl.decelCruise = True  
    elif button_type == "set":  
      # This would need to be implemented in the car controller  
      CC.cruiseControl.setCruise = True  
    elif button_type == "gap_adjust":  
      # This would need to be implemented in the car controller  
      CC.cruiseControl.gapAdjustCruise = True  
    
      
    return CC