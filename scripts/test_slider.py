import Tkinter as tk
import Image
import ImageTk
import numpy as np
import tkFileDialog

class DIP(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        self.parent = parent
        self.initUI()

    def initUI(self):
        self.parent.title("DIP Algorithms- Simple Photo Editor")
        self.pack(fill = tk.BOTH, expand = 1)

        menubar = tk.Menu(self.parent)
        self.parent.config(menu = menubar)

        self.label1 = tk.Label(self, border = 25)
        self.label2 = tk.Label(self, border = 25)
        self.label1.grid(row = 1, column = 1)
        self.label2.grid(row = 1, column = 2)

        # File Menu
        fileMenu = tk.Menu(menubar)
        menubar.add_cascade(label = "File", menu = fileMenu)

        # Menu Item for Open Image
        fileMenu.add_command(label = "Open", command = self.onOpen)

        # Basic menu
        basicMenu = tk.Menu(menubar)
        menubar.add_cascade(label = "Basic", menu = basicMenu)

        # Menu Item for image negative
        basicMenu.add_command(label = "Negative", command = self.onNeg)

        #menu for brightness
        basicMenu.add_command(label = "Brightness", command = self.onBrghtness)

    def onBrghtness(self):
        #Image Brightness Adjustment Menu callback
        brgTk=tk.Tk()
        self.brgSc = tk.Scale( brgTk, from_=-50, to=50, orient=tk.HORIZONTAL, command=self.adjBright, length=200 ,width=10, sliderlength=15)
        self.brgSc.pack(anchor=tk.CENTER)

    def adjBright(self):
        print self.brgSc #***here how can i get slider value, this callback is not being invoked***???

    def onNeg(self):
        #Image Negative Menu callback
        self.I2 = 255-self.I;
        self.im = Image.fromarray(np.uint8(self.I2))
        photo2 = ImageTk.PhotoImage(self.im)
        self.label2.configure(image=photo2)
        self.label2.image = photo2 # keep a reference!

    def setImage(self):
        self.img = Image.open(self.fn)
        self.I = np.asarray(self.img)
        l, h = self.img.size
        text = str(2*l+100)+"x"+str(h+50)+"+0+0"
        self.parent.geometry(text)
        photo = ImageTk.PhotoImage(self.img)
        self.label1.configure(image = photo)
        self.label1.image = photo # keep a reference!

    def onOpen(self):
        #Open Callback
        ftypes = [('Image Files', '*.tif *.jpg *.png')]
        dlg = tkFileDialog.Open(self, filetypes = ftypes)
        filename = dlg.show()
        self.fn = filename
        #print self.fn #prints filename with path here
        self.setImage()

    #def onError(self):
        #box.showerror("Error", "Could not open file")


def main():
    root = tk.Tk()
    DIP(root)
    root.geometry("320x240")
    root.mainloop()

if __name__ == '__main__':
    main()