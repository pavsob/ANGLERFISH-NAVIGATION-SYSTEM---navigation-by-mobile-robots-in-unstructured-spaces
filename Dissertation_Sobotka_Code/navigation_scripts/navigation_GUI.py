from Tkinter import *
import reverse_anglerfish
import potential_field
import math

# Functions
def start():
    cont_dist = float(control_distance.get())
    linear_speed = float(velocity.get())
    chosen_control_span = math.radians(float(control_span.get()))
    num_ranges = float(dist_measure_num.get())
    goal_x = float(x_coord.get())
    goal_y = float(y_coord.get())
    if (vis_checked.get() > 0):
        vis_activation = True
    else:
        vis_activation = False

    if (mode.get() == 2):
        reverse_anglerfish.run(cont_dist, linear_speed, chosen_control_span, num_ranges, goal_x, goal_y, vis_activation)
    elif(mode.get() == 1):
        potential_field.run(cont_dist, linear_speed, chosen_control_span, num_ranges, goal_x, goal_y, vis_activation)

def stop():
    reverse_anglerfish.stop()

# Main window and frames
main_window = Tk(className = 'Navigation GUI')
main_window.title('Navigation GUI')

canvas = Canvas(main_window, height= 570, width= 600, bg='#055599')
canvas.pack()
frame = Frame(canvas, bg='white', padx=25)
frame.place(x=0,y=0,relwidth = 0.95, relheight = 0.8, relx = 0.025, rely = 0.025)
bottom_frame = Frame(canvas, bg='#055599')
bottom_frame.place(relx=0.05,rely=0.87)

# Logo image
logo = PhotoImage(file = "logo.png")
#resized = logo.zoom(60)
#logo = logo.subsample(2)
label_logo = Label(canvas, image=logo, bg='#055599')
#canvas.create_image(25,240, anchor=NW,image = resized)
label_logo.place(relx=0.55, rely=0.85)

# Text
modes_text = Label(frame, text='Modes: ', font= ("Helvetica",20,"bold"), bg='white')
modes_text.grid(row = 0, column = 0, sticky = W, pady = 15)
parameters_text = Label(frame, text='Parameters: ', font= ("Helvetica",20,"bold"), bg='white')
parameters_text.grid(row = 2, column = 0, sticky = W, pady = 15)
goal_text = Label(frame, text='Goal coordinates: ', font= ("Helvetica",20,"bold"), bg='white')
goal_text.grid(row=4,column=0, sticky = W, pady = 15)
vizual_text = Label(frame, text='Visualization: ', font= ("Helvetica",20,"bold"), bg='white')
vizual_text.grid(row=6,column=0, sticky = W, pady = 15)


# Mode selection
mode_frame = Frame(frame, bg= 'white')
mode_frame.grid(row=1, sticky=W)

mode = IntVar()
pot_field_mode = Radiobutton(mode_frame, text='Classical Potential Field', bg='white', variable=mode, value=1)
progress_field_mode = Radiobutton(mode_frame, text='Reverser Anglerfish - Progress Field', bg='white', variable=mode, value=2)

pot_field_mode.grid(row=0, column=0, padx=20, sticky=W)
progress_field_mode.grid(row=0, column=1, padx=20, sticky=W)
progress_field_mode.select()

# Parameters
parameters_frame = Frame(frame, bg= 'white')
parameters_frame.grid(row = 3, sticky=W)
Label(parameters_frame, text='Control contour distance (m):', bg='white').grid(row=0, column=0, sticky=W, padx=20)
Label(parameters_frame, text='Control contour span (degrees):', bg='white').grid(row=1, column=0, sticky=W, padx=20)
Label(parameters_frame, text="Robot's velocity (m/s):", bg='white').grid(row=2, column=0, sticky=W, padx=20)
Label(parameters_frame, text='Number of obstacle diststance measurements from sensor:', bg='white').grid(row=3, column=0, sticky=W, padx=20)


control_distance = Entry(parameters_frame, width = 5)
control_distance.insert(0, 0.9)
control_distance.grid(row=0, column=1)

control_span = Entry(parameters_frame, width = 5)
control_span.insert(0, 90)
control_span.grid(row=1, column=1)

velocity = Entry(parameters_frame, width = 5)
velocity.insert(0, 0.4)
velocity.grid(row=2, column=1)

dist_measure_num = Entry(parameters_frame, width = 5)
dist_measure_num.insert(0, 72)
dist_measure_num.grid(row=3, column=1)

# Goal coordinates
goal_frame = Frame(frame, bg= 'white')
goal_frame.grid(row = 5, sticky=W)
Label(goal_frame, text='X coordinate: ', bg='white').grid(row=0, column=0, sticky=W, padx=20)
Label(goal_frame, text='Y coordinate: ', bg='white').grid(row=0, column=2, sticky=W, padx=20)

x_coord = Entry(goal_frame, width = 5)
x_coord.insert(0,10)
x_coord.grid(row=0, column=1)

y_coord = Entry(goal_frame, width = 5)
y_coord.insert(0,0)
y_coord.grid(row=0, column=3)

# Visualization
vis_checked = IntVar()
vis_button = Checkbutton(frame, text='Rviz visualization', bg='white', variable=vis_checked).grid(row=7, sticky=W, padx=20)

# Start/Stop buttons
start_button = Button(bottom_frame,fg = 'green', text = 'Start', bg='white', padx=25, pady=17, borderwidth=2, command = start)
stop_button = Button(bottom_frame,fg = 'red', text = 'Stop', bg='white',padx=25, pady=17, borderwidth=2, command = stop)
start_button.grid(row=0,column = 0)
stop_button.grid(row = 0, column=1, padx=50)

mainloop()