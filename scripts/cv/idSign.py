import numpy as np
import cv2
from matplotlib import pyplot as plt

# restore the signs into the template list 
templates = []

templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/airport.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/dangerous_curve_left.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/dangerous_curve_right.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/follow_left.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/follow_right.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/junction.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/no_bicycle.png',1))
# templates.append(cv2.imread('/home/robot/dd2419_ws/src/dd2419_perception_training/signs/template/no_heavy_truck.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/no_parking.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/no_stopping_and_parking.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/residential.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/road_narrows_from_left.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/road_narrows_from_right.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/roundabout_warning.png',1))
templates.append(cv2.imread('/home/robot/dd2419_ws/src/pras_project/scripts/cv/dd2419_perception_training/signs/template/stop.png',1))

# change the size and resolution of the templates to mka them more similar to the real detected ones
# could change this part
for index in range(len(templates)):
    templates[index] = cv2.resize(templates[index], (30,30), interpolation = cv2.INTER_AREA)
    templates[index] = cv2.GaussianBlur(templates[index],(3,3),0)
    templates[index] = cv2.GaussianBlur(templates[index],(3,3),0)
    templates[index] = cv2.GaussianBlur(templates[index],(3,3),0)
    templates[index] = cv2.GaussianBlur(templates[index],(3,3),0)
    # templates[index] = cv2.GaussianBlur(templates[index],(3,3),0)

def idSign(image):
    # positions = np.zeros(len(templates), 4, 1)
    img = cv2.resize(image, (40,40), interpolation = cv2.INTER_AREA)
    ###############################################
    edges = cv2.Canny(img,20,100, L2gradient=True)
    ###############################################
    contours = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # contour_img = cv2.drawContours(img, contours, -1, (0,255,0), 1)
    boxes = []
    tmp_locs = []
    rects = []

    if len(contours) > 0:

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < 100:
                # print('Area too small')
                continue
            # if area > large_area:
            #     # print('Area too large')
            #     continue

            perimeter = cv2.arcLength(cnt,True)
            if perimeter*perimeter/area > 30:
                continue
                
            # Find enclosing rectangle
            rect = cv2.minAreaRect(cnt)

            loc_tmp = rect[0]
            reduc = 0
            if len(tmp_locs) > 1:
                for n_box in range(len(tmp_locs)):
                    if abs(tmp_locs[n_box][0] - loc_tmp[0]) < 10 \
                    or abs(tmp_locs[n_box][1] - loc_tmp[1]) < 10:
                        reduc = 1
                        continue
            if reduc == 1:
                continue
            else:
                tmp_locs.append(loc_tmp)

            box = cv2.boxPoints(rect)
            box = np.int0(box)

            angle = rect[2]
            if abs(angle) > 30:
                continue

            # check w-h ratio
            if rect[1][1] > 1.8*rect[1][0]:
                continue
            if rect[1][0] > 1.8*rect[1][1]:
                continue
            # print("Got a contour!")
            boxes.append([box])
            rects.append(rect)

    if len(boxes) > 0:
        print("YAAAAS!! I CAN SEE A SIGN")
        # for idex in range(len(boxes)):
        #     # sub_image = img[y:y+h, x:x+w]
        #     margin = 1
        #     sub_image = image[int(rects[idex][0][1] - 0.5*rects[idex][1][1] - 3 - margin):\
        #                 int(rects[idex][0][1] + 0.5*rects[idex][1][1] - 3 ), \
        #                 int(rects[idex][0][0] - 0.5*rects[idex][1][0] - 2 ):\
        #                 int(rects[idex][0][0] + 0.5*rects[idex][1][0] - 3 )]

            # result_pic = idSign_second(sub_image)
        for idex in range(len(boxes)):
            contour_img = cv2.drawContours(img,boxes[idex],0,(0,0,255),2)

            cv2.imshow("second contour", contour_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    # return the number of the sign 
    # return sign_no


# # paint the sign
# def color_process(image):
#     # first resize, then paint and transfer to the matching part


# def sign_matching(image):























# USing Feature Matching + Homography to find Objects
# def idSign_second(image):
#     # positions = np.zeros(len(templates), 4, 1)
#     img2 = image # trainImage
#     # img2 = cv2.resize(img2, (100,100), interpolation = cv2.INTER_AREA)
#     # kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
#     # img2 = cv2.filter2D(img2, -1, kernel)
#     # img2 = cv2.resize(img2, (60,60), interpolation = cv2.INTER_AREA)

#     # alpha = 1.0 # Simple contrast control
#     # beta = 10    # Simple brightness control
#     # # Do the operation new_image(i,j) = alpha*image(i,j) + beta
#     # # Instead of these 'for' loops we could have used simply:
#     # # new_image = cv.convertScaleAbs(image, alpha=alpha, beta=beta)
#     # # but we wanted to show you how to access the pixels :)
#     # for y in range(img2.shape[0]):
#     #     for x in range(img2.shape[1]):
#     #         for c in range(img2.shape[2]):
#     #             img2[y,x,c] = np.clip(alpha*img2[y,x,c] + beta, 0, 255)


#     result_mat = np.zeros((len(templates),1))

#     for idex in range(len(templates)):
#         # Feature Matching + Homography to find Objects
#         MIN_MATCH_COUNT = 100
#         img1 = templates[idex]          # queryImage

#         # Initiate SIFT detector
#         sift = cv2.xfeatures2d.SIFT_create()

#         # find the keypoints and descriptors with SIFT
#         kp1, des1 = sift.detectAndCompute(img1,None)
#         kp2, des2 = sift.detectAndCompute(img2,None)
            
#         # FLANN_INDEX_KDTREE = 0
#         # index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
#         # # FLANN_INDEX_LSH = 6
#         # # index_params= dict(algorithm = FLANN_INDEX_LSH,
#         # #            table_number = 6, # 12
#         # #            key_size = 12,     # 20
#         # #            multi_probe_level = 1) #2

#         # search_params = dict(checks = 20)

#         # flann = cv2.FlannBasedMatcher(index_params, search_params)

#         # matches = flann.knnMatch(des1,des2,k=2)

#         bf = cv2.BFMatcher()
#         matches = bf.knnMatch(des1,des2, k=2)

#             # store all the good matches as per Lowe's ratio test.
#         good = []

#         for m,n in matches:
#             if m.distance < 0.95*n.distance:
#                 good.append(m)

#         if len(good)>MIN_MATCH_COUNT:
#             src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
#             dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

#             M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,3.0)
#             matchesMask = mask.ravel().tolist()
            
#             h,w = img1.shape
#             pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
#             dst = cv2.perspectiveTransform(pts,M)

#             img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

#         else:
#             print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
#             matchesMask = None


#         draw_params = dict(matchColor = (0,255,0), # draw matches in green color
#                             singlePointColor = None,
#                             matchesMask = matchesMask, # draw only inliers
#                             flags = 2)

#         img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)

#         # if len(good) > 3:
#         plt.imshow(img3, 'gray'),plt.show()

#         result_mat[idex] = len(good)

#     tmp_num = np.argmax(result_mat)

    # if(result_mat[tmp_num] > 6):    
    #     cv2.imshow("Extraction", image)
    #     cv2.imshow("Corrsponding template", templates[tmp_num])
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()


    # print 





# Using cv2.matchTemplate
# def idSign_second(image):
#     global templates

#     image = cv2.resize(image, (35,35), interpolation = cv2.INTER_AREA)

#     raw_image = image.copy()

#     values = []
    
#     for num in range(len(templates)):
#         # loop all the templates
#         template = templates[num]

#         result = cv2.matchTemplate(image, templates[num], cv2.TM_CCOEFF_NORMED)
#         (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)

#         values.append(maxVal)

#     threshold = 0.2

#     if max(values) > threshold:
#         print(max(values))
#         cv2.imshow("Matched image", image)
#         tmp_num = values.index(max(values))
#         print(tmp_num)
#         cv2.imshow("Corrsponding template", templates[tmp_num])
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()


#         return tmp_num
#         # rospy.sleep(2)