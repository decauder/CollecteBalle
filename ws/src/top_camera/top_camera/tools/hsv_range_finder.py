import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
from PIL import Image, ImageTk


class HSVRangeFinder:
    def __init__(self, img):
        self.img = img
        self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        self.min_h = 0
        self.max_h = 179
        self.min_s = 0
        self.max_s = 255
        self.min_v = 0
        self.max_v = 255

        self.create_widgets()
        self.update_threshold()

    def create_widgets(self):
        self.root = tk.Tk()
        self.root.title('HSV Range Finder')

        # Create sliders for min and max values for each channel
        self.h_label = tk.Label(self.root, text='Hue')
        self.h_min_slider = tk.Scale(self.root, from_=0, to=179, orient=tk.HORIZONTAL,
                                     length=200, command=self.on_value_change)
        self.h_max_slider = tk.Scale(self.root, from_=0, to=179, orient=tk.HORIZONTAL,
                                     length=200, command=self.on_value_change)
        self.s_label = tk.Label(self.root, text='Saturation')
        self.s_min_slider = tk.Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL,
                                     length=200, command=self.on_value_change)
        self.s_max_slider = tk.Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL,
                                     length=200, command=self.on_value_change)
        self.v_label = tk.Label(self.root, text='Value')
        self.v_min_slider = tk.Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL,
                                     length=200, command=self.on_value_change)
        self.v_max_slider = tk.Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL,
                                     length=200, command=self.on_value_change)

        # Set default slider values
        self.h_min_slider.set(self.min_h)
        self.h_max_slider.set(self.max_h)
        self.s_min_slider.set(self.min_s)
        self.s_max_slider.set(self.max_s)
        self.v_min_slider.set(self.min_v)
        self.v_max_slider.set(self.max_v)

        # Place the widgets in the window
        self.h_label.grid(row=0, column=0, pady=5)
        self.h_min_slider.grid(row=0, column=1, pady=5)
        self.h_max_slider.grid(row=0, column=2, pady=5)
        self.s_label.grid(row=1, column=0, pady=5)
        self.s_min_slider.grid(row=1, column=1, pady=5)
        self.s_max_slider.grid(row=1, column=2, pady=5)
        self.v_label.grid(row=2, column=0, pady=5)
        self.v_min_slider.grid(row=2, column=1, pady=5)
        self.v_max_slider.grid(row=2, column=2, pady=5)

        # Create a canvas to display the original and thresholded images
        self.canvas = tk.Canvas(self.root, width=self.img.shape[1], height=self.img.shape[0])
        self.canvas.grid(row=0, column=3, rowspan=3, padx=5)

    def on_value_change(self, value):
        self.min_h = self.h_min_slider.get()
        self.max_h = self.h_max_slider.get()
        self.min_s = self.s_min_slider.get()
        self.max_s = self.s_max_slider.get()
        self.min_v = self.v_min_slider.get()
        self.max_v = self.v_max_slider.get()
        self.update_threshold()

    def update_threshold(self):
        # Create a mask based on the current slider values
        lower = np.array([self.min_h, self.min_s, self.min_v])
        upper = np.array([self.max_h, self.max_s, self.max_v])
        mask = cv2.inRange(self.hsv_img, lower, upper)

        # Apply the mask to the original image
        result = cv2.bitwise_and(self.img, self.img, mask=mask)

        # Display the original and thresholded images on the canvas
        img_rgb = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img_rgb)
        img_tk = ImageTk.PhotoImage(img_pil)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=img_tk)
        self.canvas.image = img_tk  # Keep a reference to prevent garbage collection

    def run(self):
        # Start the GUI
        self.root.mainloop()
