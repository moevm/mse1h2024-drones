from tkinter import Button, Label, Tk
from tkvideo import tkvideo
from sys import argv


def start_demo():
    print("start demonstration")


def end_demo():
    print("end demonstration")


def emergency():
    print("emergency stop")


window = Tk()
window.title("aruco tracker")

# buttons init
button_start = Button(window, text="Начать демонстрацию", width=20, 
                      height=5, bg='white', fg='black', command=start_demo)
button_end = Button(window, text="Закончить демонстрацию", width=20,
                    height=5, bg='white', fg='black', command=end_demo)
button_emergency = Button(window, text="Аварийная остановка", width=20, 
                          height=5, bg='white', fg='black',command=emergency)

# video init
my_label = Label(window)
my_label.pack()

video_name = "video.mp4" if len(argv) == 1 else argv[1]
player = tkvideo(video_name, my_label, loop=1, size=(600, 480))
player.play()

button_start.pack()
button_end.pack()
button_emergency.pack()

window.mainloop()
