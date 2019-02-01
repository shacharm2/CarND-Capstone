import keras

from keras.applications import resnet50, inception_v3, vgg16, resnet50, mobilenet
from keras.layers import Dense, GlobalAveragePooling2D, Flatten, Input, Dropout, Conv2D, Activation, MaxPooling2D, MaxPool2D
from keras.models import Model, Sequential
from keras.optimizers import Adam, SGD
from keras.preprocessing.image import ImageDataGenerator
from keras import regularizers

import cv2
import ipdb

data_path = '../../../../../udacity-simulator-data/training-images'

def build_model(num_classes, input_shape):
    #base_model = vgg16.VGG16(weights='imagenet', include_top=False, pooling='max')
    base_model = resnet50.ResNet50(weights='imagenet', include_top=False, pooling='max', input_shape=input_shape)
    # base_model = mobilenet.MobileNet(weights='imagenet', include_top=False, pooling='max', input_shape=input_shape)

    for layer in base_model.layers:
	    layer.trainable = False    
    x = base_model.output

#     x = Conv2D(32, kernel_size=3, activation='relu')(x)
#     x = Dropout(0.5)(x)
#     x = MaxPool2D(pool_size=3, strides=2, padding='same')(x)

#     x = Conv2D(32, kernel_size=3, activation='relu')(x)
#     x = Dropout(0.5)(x)
#     x = MaxPool2D(pool_size=3, strides=2, padding='same')(x)
#     x = Dense(128, activation='relu')(x)
#     x = Dense(32, activation='relu')(x)
#     x = Dense(16, activation='relu')(x)
    x = Dense(num_classes, activation='softmax')(x)

    model = Model(inputs=base_model.input, outputs=x)

    model.compile(optimizer=Adam(lr=1e-2), loss='categorical_crossentropy', metrics=['accuracy'])
    return model

def build_model(num_classes, input_shape):
    model = Sequential()

    model.add(Conv2D(32, kernel_size=3, activation='relu', input_shape=input_shape))
    model.add(MaxPool2D(pool_size=3, strides=2, padding='same'))
    model.add(Conv2D(32, kernel_size=3, activation='relu'))
    model.add(MaxPool2D(pool_size=3, strides=2, padding='same'))    
    model.add(Dropout(0.5))

    model.add(Conv2D(32, kernel_size=3, activation='relu', kernel_regularizer=regularizers.l2(0.01)))
    model.add(MaxPool2D(pool_size=3, strides=2, padding='same'))
    model.add(Conv2D(32, kernel_size=3, activation='relu', kernel_regularizer=regularizers.l2(0.01)))
    model.add(MaxPool2D(pool_size=3, strides=2, padding='same'))
    model.add(Dropout(0.5))

    model.add(Flatten())
    model.add(Dense(num_classes, activation='softmax'))
    model.compile(optimizer=Adam(lr=1e-4), loss='categorical_crossentropy', metrics=['accuracy'])
    return model



#abs_path = '/home/udacity/udacity/udacity-simulator-data/carla-images'
# abs_path = '../../../../../udacity-simulator-data/training-images/train'
# test_path = '../../../../../udacity-simulator-data/training-images/test'

abs_path = '{}/train'.format(data_path)
test_path = '{}/test'.format(data_path)
# carla_path = '/home/udacity/udacity/udacity-simulator-data/carla-images-cropped/'
filepath = './model_files/classifier_model_weights.{epoch:02d}-{val_loss:.2f}.hdf5'
model_checkpoint = keras.callbacks.ModelCheckpoint(filepath, monitor='val_loss', verbose=0,
                                                    save_best_only=True, save_weights_only=False, 
                                                    mode='auto', period=1)


datagen_train = ImageDataGenerator(
        rotation_range=20,
        width_shift_range=0.2,
        height_shift_range=0.2,
        rescale=1./255,
        # shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True,
        fill_mode='nearest')

#train_datagen = ImageDataGenerator(rescale=1./255, shear_range=0.2, zoom_range=0.2, horizontal_flip=True)


# https://medium.com/datadriveninvestor/keras-imagedatagenerator-methods-an-easy-guide-550ecd3c0a92
x = cv2.imread('{}/test/2/left0450.jpg'.format(data_path))


classes = ['0', '1', '2']
train_generator = datagen_train.flow_from_directory(directory=abs_path,
    classes=classes,
    class_mode='categorical',
    batch_size=32,
    target_size=(x.shape[0],x.shape[1]))


datagen_val = ImageDataGenerator(
        rotation_range=10,
        width_shift_range=0.2,
        height_shift_range=0.2,
        rescale=1./255,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True,
        fill_mode='nearest')

val_generator = datagen_val.flow_from_directory(directory=test_path, classes=classes, class_mode='categorical', batch_size=32, target_size=(x.shape[0], x.shape[1]))

model = build_model(num_classes=len(classes), input_shape=x.shape)
# STEP_SIZE_TRAIN=train_generator.n//train_generator.batch_size
model.fit_generator(train_generator,
    steps_per_epoch=train_generator.n//train_generator.batch_size,
    validation_data=val_generator,
    validation_steps=10,
    callbacks=[model_checkpoint],
    epochs=75)

ipdb.set_trace()
