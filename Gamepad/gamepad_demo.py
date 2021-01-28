#Game pad Demo script

#import evdev
from evdev import InputDevice, categorize, ecodes

# ***** MAIN ***** #
if __name__ == "__main__":
    
    #create object gamepad
    gamepad = InputDevice('/dev/input/event2')


    #button code variables
    A_press = 289
    B_press = 290
    X_press = 288
    Y_press = 291

    Start_press = 297
    Select_press = 296

    Right_trig = 293
    Left_trig = 292

    #print out device info
    #print(gamepad)

    #evdev
    for event in gamepad.read_loop():

        #on = True
        #print("Reading Gamepad")
        
        #while(on):
        #event = gamepad.read()
        
        #print(categorize(event))
        #print(event)

        if event is None:
            print("-")

        else:
            #Sort event into action:
            if event.type == ecodes.EV_KEY:
                #Button
                if event.value==1:
                    if event.code==A_press:
                        print("A")
                    elif event.code==B_press:
                        print("B")
                    elif event.code==X_press:
                        print("X")
                    elif event.code==Y_press:
                        print("Y")
                    elif event.code==Start_press:
                        print("Start")
                        on = False
                        print("Turning off")
                    elif event.code==Select_press:
                        print("Select")
                    elif event.code==Right_trig:
                        print("Right Trigger")
                    elif event.code==Left_trig:
                        print("Left Trigger")
                
            
            elif event.code==1 and event.type==3:
                #Up or down buttons
                if event.value==255:
                    print("hold down")
                elif event.value==0:
                    print("hold up")
                elif event.value==127:
                    print("hold off")
            elif event.code==0 and event.type==3:
                #left or right buttons
                if event.value==0:
                    print("hold left")
                elif event.value==255:
                    print("hold right")
                elif event.value==127:
                    print("hold off")

