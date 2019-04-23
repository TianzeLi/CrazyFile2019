#!/usr/bin/env python
from __future__ import print_function

import sys, os, shutil
import numpy as np

from Tkinter import *
import ttk
import Tkinter as tk
import tkFont
from PIL import ImageTk as itk
from PIL import Image
from functools import partial

class imageSorter:
    def __init__(self,imgFolder, imgSortPath, classNames, buttonGrouping, view_size = (500,500)):
        self.imgFolder = imgFolder
        self.imgSortPath = imgSortPath
        self.classNames = classNames
        self.view_size = view_size
        self.buttonGrouping = buttonGrouping

        self.currImgPath = ''
        self.imgs = []

        self.NImages = len(os.listdir(self.imgFolder))
        self.HandledImages = 0

        self.root = tk.Tk()
        self.img_label = tk.Label(self.root)
        self.img_label.grid(column=1, row=0)
        #self.img_label.pack()

        self.progress=ttk.Progressbar(self.root,orient=HORIZONTAL,length=100,mode='determinate')
        self.progress.grid(column=2, row=0)
        #self.progress.pack()

        self.imgs = iter(os.listdir(self.imgFolder))

        helvLarge = tkFont.Font(family='Helvetica', size=20, weight='bold')
        self.next_btn = tk.Button(self.root, text='Welcome!', command=self.next_img, font=helvLarge)
        self.next_btn['text'] = str(self.HandledImages) + '/' + str(self.NImages)
        self.next_btn.grid(column=2, row=0)
        #self.next_btn.pack()

        classButtons = ['1','2','3','4','5','6','7','8','9','0','q','w','e','r','t','y','u','i','o','p']
        helv = tkFont.Font(family='Helvetica', size=15)
        cols = [0,0,1,1,1,1,1,1,1,2,2,2,2,2,2,3,3] #because im lazy
        rows = np.add([0,1,0,1,2,3,4,5,6,0,1,2,3,4,5,0,1],3)
        for i in range(0,len(self.classNames)):
            class_btn = tk.Button(self.root, text=classButtons[i] + '. ' + self.classNames[i], font=helv, command= partial(self.sort_img,i))
            class_btn.grid(column=cols[i], row=rows[i])
            #class_btn.pack()


        self.frame = Frame(self.root)
        self.frame.bind("<KeyPress>", self.keydown)
        self.frame.grid(column=1, row=0)
        #frame.pack()
        self.frame.focus_set()

        self.next_img()

        self.root.mainloop()

    def next_img(self):
        if self.HandledImages != self.NImages:
            self.currImgPath = self.imgFolder + next(self.imgs)
            img = Image.open(self.currImgPath)
            img = img.resize(self.view_size, Image.ANTIALIAS)
            self.img_label.img = itk.PhotoImage(img)
            self.img_label.config(image=self.img_label.img)
            self.frame.focus_set()
        else:
            print('All done!')
            os.rmdir(self.imgFolder)
            self.root.destroy()


    def keydown(self, e):
        c = str(e.char)
        if c == ' ':
            self.next_img()
        else:
            classButtons = ['1','2','3','4','5','6','7','8','9','0','q','w','e','r','t','y','u','i','o','p']
            ind = classButtons.index(c)
            self.sort_img(ind)

    def sort_img(self, sort_class):
        fromFile = self.currImgPath
        fileNameSplit = fromFile.split('/')
        imgname = fileNameSplit[len(fileNameSplit)-1] #actual name of file

        toFile = self.imgSortPath + self.classNames[sort_class] +'/'+imgname
        shutil.move(fromFile, toFile)

        self.HandledImages += 1
        self.progress['value'] = np.round(float(self.HandledImages)/self.NImages)
        self.next_btn['text'] = str(self.HandledImages) + '/' + str(self.NImages)
        print('Moved file: ' + fromFile)
        print('To: ' + toFile)

        self.next_img()


def create_folders(imgSortPath, classNames):
    if os.path.exists(imgSortPath) == False:
        os.mkdir(imgSortPath)
    for i in range(0, len(classNames)):
        if os.path.exists(imgSortPath + classNames[i]) == False:
            os.mkdir(imgSortPath + classNames[i])


def main(args):
    print("running...")
    imgFolderName = str(args[1])#'imgs/junction'

    classNames = ['airport', 'residential','follow_left','follow_right','no_bicycle','no_heavy_truck','no_parking','no_stopping_and_parking','stop','dangerous_curve_left','dangerous_curve_right','junction','road_narrows_from_left','road_narrows_from_right','roundabout_warning','z_crap']
    classNames.append('unsure')
    buttonGrouping = [2,7,6,2]

    imgFolder = os.getcwd() + '/' + imgFolderName + '/'
    imgSortPath = os.getcwd() + '/sorted/'

    create_folders(imgSortPath, classNames)
    imageSorter(imgFolder, imgSortPath, classNames, buttonGrouping)

if __name__ == '__main__':
    main(sys.argv)
