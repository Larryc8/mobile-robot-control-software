import numpy as np
import cv2 as cv
import cv2
from matplotlib import pyplot as plt
# import matplotlib.pyplot as plt

from PyQt5.QtCore import QObject, QThread


        # orb = cv.ORB_create(
        #     nfeatures=500,
        #     scaleFactor=1.2,
        #     nlevels=8,
        #     edgeThreshold=31,
        #     firstLevel=0,
        #     WTA_K=2,
        #     scoreType=cv.ORB_HARRIS_SCORE,
        #     patchSize=31,
        #     fastThreshold=20,
        # )

class ImageMatcheChecker():
    def __init__(self) -> None:
        super().__init__()

    def run(self) -> None:
        pass

    def s(self):
        # Crear ORB
        img1 = cv.imread('./reference_images/Untitled2_mod.jpg',cv.IMREAD_GRAYSCALE)          # queryImage
        # img2 = cv.imread('./reference_images/Untitled_mod_leaf.png',cv.IMREAD_GRAYSCALE) # trainImage
        img2 = cv.imread('./reference_images/Untitled1_mod_leaf.png',cv.IMREAD_GRAYSCALE) # trainImage
        # img2 = cv.imread('./reference_images/Untitled1.jpg',cv.IMREAD_GRAYSCALE) # trainImage
        equalized_hist1 = cv2.equalizeHist(img1)
        equalized_hist2 = cv2.equalizeHist(img2)

        # Contrast Limited Adaptive Histogram Equalization
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        equalized_clache1 = clahe.apply(img1)
        equalized_clache2 = clahe.apply(img2)
        # Initiate ORB detector
        orb = cv.ORB_create(nfeatures=500, patchSize=9, edgeThreshold=5)
        # find the keypoints and descriptors with ORB
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)
        print(f'des {des1[0]} len by des {len(des1[0])}')

        bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
        # Match descriptors.
        matches = bf.match(des2, des1)
        print(f' matches: {len(matches)}, des1: {len(des1)} des2: {len(des2)} ---> {len(matches)/len(des1)}')
        # print(des1[0], des2[0])
        print('---------------'*3)
        # print(des1[1], des2[1])
        print('---------------'*3)
        # print(matches[0])

        matches = sorted(matches, key=lambda x: x.distance)

        print(matches[6].distance)

        plt.figure(figsize=(20, 10))
        plt.imshow(img1)
        plt.title("ORB Feature Matching")
        plt.show()

        plt.imshow(img2)
        plt.title("ORB Feature Matching")
        plt.show()
        
        print('!!!!!!!!!!!!!!!!!!!!!!')
        # kp1, des1 = orb.detectAndCompute(equalized_hist1,None)
        # kp2, des2 = orb.detectAndCompute(equalized_hist2,None)

        # matches = bf.match(des2, des1)
        # print(f' matches: {len(matches)}, des1: {len(des1)} des2: {len(des2)} ---> {len(matches)/len(des1)}')
        # print('!!!!!!!!!!!!!!!!!!!!!!')

        # kp1, des1 = orb.detectAndCompute(equalized_clache1,None)
        # kp2, des2 = orb.detectAndCompute(equalized_clache2,None)

        # matches = bf.match(des2, des1)
        # print(f' matches: {len(matches)}, des1: {len(des1)} des2: {len(des2)} ---> {len(matches)/len(des1)}')

    def s1(self):
        # Load images
        img1_path = './reference_images/Untitled3_mod.jpg'
        img2_path = './reference_images/Untitled2_mod.jpg'
        img1 = cv2.imread(img1_path, cv2.IMREAD_GRAYSCALE)
        img2 = cv2.imread(img2_path, cv2.IMREAD_GRAYSCALE)
        
        if img1 is None or img2 is None:
            print("Could not open or find the images!")
            return


        img1 = cv2.equalizeHist(img1)
        img2 = cv2.equalizeHist(img2)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        img1 = clahe.apply(img1)
        img2 = clahe.apply(img2)
        
        # Initialize ORB detector
        orb = cv2.ORB_create(nfeatures=500)
        
        # Find keypoints and descriptors
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)
        
        # Create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        # Match descriptors using KNN
        matches = bf.knnMatch(des1, des2, k=2)
        print(matches)
        
        # Apply ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < 0.8 * n.distance:
                good_matches.append([m])
                print(f'm.distance {m.distance}  n.distance {n.distance}')

        print(f'good matches: {len(good_matches)}')
        
        # Draw matches
        img_matches = cv2.drawMatchesKnn(
            img1, kp1, 
            img2, kp2, 
            good_matches, 
            None, 
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
            matchColor=(0, 255, 0)
        )
        
        # Display results
        plt.figure(figsize=(20, 10))
        plt.imshow(img_matches)
        plt.title("ORB Feature Matching with KNN and Ratio Test")
        plt.axis('off')
        plt.show()
        
        print(f"Total matches: {len(matches)}")
        print(f"Good matches after ratio test: {len(good_matches)}")

    def s2(self):
        pass


# Example usage


if __name__ == "__main__":
    ex = ImageMatcheChecker()
    # ex.s()
    ex.s1()
