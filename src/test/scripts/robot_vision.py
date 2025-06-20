import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

from PyQt5.QtCore import QObject, QThread


class ImageMatcheChecker(QThread):
    def __init__(self) -> None:
        super().__init__()

    def run(self) -> None:
        pass

    def s(self):
        # Crear ORB
        orb = cv.ORB_create()

        # Detectar y calcular descriptores
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)

        # Create an ORB object
        orb = cv.ORB_create()

        # ORB object with custom parameters
        orb = cv.ORB_create(
            nfeatures=500,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            firstLevel=0,
            WTA_K=2,
            scoreType=cv.ORB_HARRIS_SCORE,
            patchSize=31,
            fastThreshold=20,
        )

        # Configurar FLANN para ORB
        index_params = dict(
            algorithm=6, table_number=12, key_size=20, multi_probe_level=2
        )
        search_params = dict(checks=50)

        flann = cv.FlannBasedMatcher(index_params, search_params)

        # Convertir a tipo compatible (float32 no es necesario para LSH)
        matches = flann.knnMatch(des1, des2, k=2)

        # Ratio test
        good = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good.append(m)
                img1 = cv.imread("box.png", cv.IMREAD_GRAYSCALE)  # queryImage
                img2 = cv.imread("box_in_scene.png", cv.IMREAD_GRAYSCALE)  # trainImage

        # Initiate SIFT detector
        sift = cv.ORB_create()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(img1, None)
        kp2, des2 = sift.detectAndCompute(img2, None)

        # FLANN parameters
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)  # or pass empty dictionary

        flann = cv.FlannBasedMatcher(index_params, search_params)

        matches = flann.knnMatch(des1, des2, k=2)

        # Need to draw only good matches, so create a mask
        matchesMask = [[0, 0] for i in range(len(matches))]

        # ratio test as per Lowe's paper
        for i, (m, n) in enumerate(matches):
            if m.distance < 0.7 * n.distance:
                matchesMask[i] = [1, 0]
                # Do somethong

        # draw_params = dict(matchColor = (0,255,0),
        #                    singlePointColor = (255,0,0),
        #                    matchesMask = matchesMask,
        #                    flags = cv.DrawMatchesFlags_DEFAULT)

        # img3 = cv.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)

        # plt.imshow(img3,),plt.show()
