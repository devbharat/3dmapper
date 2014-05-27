LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := octomath_user
LOCAL_SRC_FILES := octomath_user.cpp
LOCAL_SHARED_LIBRARIES := octomath octomap opencv_java
LOCAL_LDLIBS +=  -llog -ldl


include $(BUILD_SHARED_LIBRARY)
