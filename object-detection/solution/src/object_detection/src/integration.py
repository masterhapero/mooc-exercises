#!/usr/bin/env python
# coding: utf-8

# In[ ]:


def DT_TOKEN():
    # todo change this to your duckietown token
    dt_token = "dt1-3nT8KSoxVh4MdNoZ2qiQRvtfhbaFAfMnXzJpX9qY6GUXwBa-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfXp9GBV23KN2uUQNkenUeGSjQFoUiHRa3L"
    return dt_token

def MODEL_NAME():
    # todo change this to your model's name that you used to upload it on google colab.
    # if you didn't change it, it should be "yolov5"
    return "yolov5"


# In[ ]:


def NUMBER_FRAMES_SKIPPED():
    # todo change this number to drop more frames
    # (must be a positive integer)
    return 0


# In[ ]:


# `class` is the class of a prediction
def filter_by_classes(clas):
    # Right now, this returns True for every object's class
    # Change this to only return True for duckies!
    # In other words, returning False means that this prediction is ignored.
    print(f'filter {clas}')
    if int(clas) == 0:
        print("Class true")
        return True
    return False


# In[ ]:


# `scor` is the confidence score of a prediction
def filter_by_scores(scor):
    # Right now, this returns True for every object's confidence
    # Change this to filter the scores, or not at all
    # (returning True for all of them might be the right thing to do!)
    print(f'scor {scor}')
    if scor < 0.2:
        return False
    print("score true")
    return True


# In[ ]:


# `bbox` is the bounding box of a prediction, in xyxy format
# So it is of the shape (leftmost x pixel, topmost y pixel, rightmost x pixel, bottommost y pixel)
def filter_by_bboxes(bbox):
    # Like in the other cases, return False if the bbox should not be considered.
    print(f'bbox {bbox[0]} x {bbox[1]} x {bbox[2]} x {bbox[3]}')
    if bbox[3] > 235.0:
        print("bbox true")
        return True

    return False

