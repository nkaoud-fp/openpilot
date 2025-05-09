# In selfdrive/frogpilot/auto_resume_variables.py  
  
# Global variable to track resume button press  
send_resume_button = False  
  
def set_resume_button(value):  
    global send_resume_button  
    send_resume_button = value  
  
def get_resume_button():  
    global send_resume_button  
    return send_resume_button  
  
def reset_resume_button():  
    global send_resume_button  
    send_resume_button = False