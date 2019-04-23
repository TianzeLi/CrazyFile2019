#!/usr/bin/env python
from __future__ import print_function

import sys, os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from keras.models import Sequential, load_model
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense
from keras.preprocessing.image import ImageDataGenerator, array_to_img, img_to_array, load_img
from keras.utils import plot_model
from keras.callbacks import Callback, ModelCheckpoint, EarlyStopping

#dataSetFilePath = os.getcwd() + '/src/PRAS_Gr6/perc_ch1/data/JulesSet/'
dataSetFilePath = '/home/s/i/simoneds/dd2419_ws/src/perc_ch1/data/SimonSet/' #folder to find data for training and validation
dataSaveFolder = 'models/' #folder to save models and weight
dataEndTestFolder = '/home/s/i/simoneds/dd2419_ws/rvizimages/'
dataEndTestFolder = '/home/s/i/simoneds/dd2419_ws/src/perc_ch1/data/SimonSet/Circles/stop/'


def train_model(classSetName, nClasses, W, H, batch_size=100, max_epochs = 100, version = 0, validation_split=0, min_val_acc=0.98):
    print('Training model for: ' + classSetName)

    #Create neural network structure:
    model = Sequential()
    convSize = [32, 64, 128, 32] #size for the convolutional layers
    n_conv = len(convSize)
    #Add convolution layers:
    for i in range(0,n_conv):
        if i == 0: #first layer needs an added input layer
            model.add(Conv2D(convSize[i], (3, 3), padding='same', activation='relu', input_shape=(W, H, 3)))
        else:
            model.add(Conv2D(convSize[i], (3, 3), padding='same', activation='relu'))
        model.add(Conv2D(convSize[i], (3,3), activation='relu'))
        model.add(MaxPooling2D(pool_size = (2,2)))
        model.add(Dropout(0.1))
    #Add flattening layers
    model.add(Flatten())
    model.add(Dense(128, activation='relu'))
    model.add(Dropout(0.5))
    #Add output layers
    model.add(Dense(nClasses, activation='softmax'))

    #Compile model
    model.compile(loss='mean_squared_error',
              optimizer='rmsprop',
              metrics=['accuracy'])

    #Set option for stopping when high enough accuracy:
    checkPointString = dataSaveFolder + 'CHKPT_' + classSetName + str(version) + '.h5'
    callbacks = [
    EarlyStoppingByAccuracy(monitor='val_acc', value=min_val_acc, verbose=1),
    ModelCheckpoint(checkPointString, monitor='val_acc', save_best_only=True, verbose=0)]

    #Depending on if there is a set validation data set or not:
    if validation_split == 0:
        #Set training options:
        train_datagen = ImageDataGenerator(
                rotation_range=20,
                width_shift_range=0.2,
                height_shift_range=0.2,
                rescale=1./255,
                shear_range=0.2,
                zoom_range=0.2,
                horizontal_flip=False,
                fill_mode='nearest')

        train_generator = train_datagen.flow_from_directory(
            dataSetFilePath + 'Training_Set/' + classSetName,
            target_size = (W, H),
            batch_size = batch_size,
            classT_mode = 'categorical')

        #Train model:
        #Set testing optins:
        test_datagen = ImageDataGenerator(rescale=1./255)
        validation_generator = test_datagen.flow_from_directory(
            dataSetFilePath + 'Test_Set/' + classSetName,
            target_size = (W, H),
            batch_size = batch_size,
            class_mode = 'categorical')


        #Train model:
        model.fit_generator(
            train_generator,
            steps_per_epoch = np.round(1000 // batch_size),
            epochs = max_epochs,
            validation_data = validation_generator,
            validation_steps = np.round(1000 // batch_size),
            callbacks = callbacks)
    else: #Validation data not set manually
        #Set training options:
        train_datagen = ImageDataGenerator(
                rotation_range=20,
                width_shift_range=0.2,
                height_shift_range=0.2,
                rescale=1./255,
                shear_range=0.2,
                zoom_range=0.2,
                horizontal_flip=False,
                validation_split = validation_split,
                fill_mode='nearest')

        train_generator = train_datagen.flow_from_directory(
            dataSetFilePath + classSetName,
            target_size = (W, H),
            batch_size = batch_size,
            class_mode = 'categorical',
            subset = 'training')

        validation_generator = train_datagen.flow_from_directory(
            dataSetFilePath + classSetName,
            target_size = (W, H),
            batch_size = batch_size,
            class_mode = 'categorical',
            subset = 'validation')

        #Train model:
        model.fit_generator(
            train_generator,
            steps_per_epoch = np.round(train_generator.samples // batch_size),
            epochs = max_epochs,
            validation_data = validation_generator,
            validation_steps = np.round(validation_generator.samples // batch_size),
            callbacks = callbacks)

    #Save resulting weights and system:
    saveString = dataSaveFolder + classSetName + str(version) + '.h5'
    print('Saving model to: ' + saveString)
    model.save(saveString)

class EarlyStoppingByAccuracy(Callback):
    def __init__(self, monitor='val_acc', value=0.95, verbose=0):
        super(Callback, self).__init__()
        self.monitor = monitor
        self.value = value
        self.verbose = verbose

    def on_epoch_end(self, epoch, logs={}):
        current = logs.get(self.monitor)
        if current is None:
            warnings.warn("Early stopping requires %s available!" % self.monitor, RuntimeWarning)

        if current >= self.value:
            if self.verbose > 0:
                print("Epoch %05d: early stopping THR" % epoch)
            self.model.stop_training = True

def test_model(classSetName, version):
    modelTestName = classSetName + str(version) + '.h5'
    model = load_model(dataSaveFolder + modelTestName)
    #plot_model(model, to_file='model.png')

    if int(version) <= 4:
        if classSetName == 'Rectangles':
            classNames = ['airport', 'residential', 'z_crap']
        elif classSetName == 'Circles':
            classNames = ['follow_left','follow_right','no_bicycle','no_heavy_truck','no_parking','no_stopping_and_parking','stop','z_crap']
        elif classSetName == 'Triangles':
            classNames = ['dangerous_curve_left','dangerous_curve_right','junction','road_narrows_from_left','road_narrows_from_right','roundabout_warning','z_crap']
    else:
        if classSetName == 'Rectangles':
            classNames = ['airport', 'residential']
        elif classSetName == 'Circles':
            classNames = ['follow_left','follow_right','no_bicycle','no_heavy_truck','no_parking','no_stopping_and_parking','stop']
        elif classSetName == 'Triangles':
            classNames = ['dangerous_curve_left','dangerous_curve_right','junction','road_narrows_from_left','road_narrows_from_right','roundabout_warning']

    ld = os.listdir(dataEndTestFolder)

    for i in range(0, len(ld)):
        img = mpimg.imread(dataEndTestFolder + ld[i])
        img2 = np.expand_dims(np.array(img),0)
        pred = model.predict(img2)
        imgplot = plt.imshow(img)
        p = np.where(pred == np.amax(pred))[1][0]
        plt.title(classNames[p])
        plt.show()

def main(args):
    print("running...")

    train = 0
    n = 0 #circles, rectangles or triangles?
    version = 4 #for weight file name
    #version 2: JulesSet
    #version 3: SimonSet, with z_crap
    #version 4: SimonSet, without z_crap

    shapes = ['Circles','Rectangles','Triangles']
    numOfSigns = [7,2,6]

    classSetName = shapes[n] #must correspond t training set folders
    nClasses = numOfSigns[n]




    if train == 1:
        train_model(classSetName, nClasses, 64, 64, version=version, validation_split = 0.2, min_val_acc = 0.95)
    else:
        test_model(classSetName, version)


if __name__ == '__main__':
    main(sys.argv)
