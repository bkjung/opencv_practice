#!/usr/bin/env python
import numpy as np
import cv2, cv, sys, time

if __name__ == "__main__":

    # if(len(sys.argv) !=6):
    #     print("Wrong input")

    board_w=(12) # number of horizontal corners
    board_h=(9) # number of vertical corners
    n_boards=1
    board_n = board_w*board_h
    board_sz = (board_w, board_h)

    #creation of memory storage
    image_pnts=np.array(n_board*board_n,2,cv2.CV_32FC1)
    obj_pnts=np.array(n_board*board_n,3,cv.CV_32FC1)
    points_count=np.array(n_board,1,cv2.CV_32SC1)
    intrinsic_matrix=np.array(3,3,cv2.CV_32FC1)
    distortion_coefficient=np.array(5,1,cv2.CV_32FC1)

    i=0
    z=0 #to print number of frames
    successes=0
    cap = cv2.VideoCapture(0)


    while cap.isOpened():
        #capturing required number of views
        while successes<n_boards:
            ret, frame= cap.read()
            image = cv2.cv.iplimage(frame)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            found=0
            (found, corners) = cv2.FindChessboardCorners(image, board_sz, cv2.CV_CALIB_CB_ADAPTIVE_THRESH| cv2.CV_CALIB_CB_FILTER_QUADS)
            #Get Subpixel accuracy on those corners
            corners = cv2.FindCornerSubPix(gray_image, corners, (11,11), (-1,-1),(cv2.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER,30,0.1) )
            
            #if got a good image, draw chess board
            if found==1:
                print("found frame number {0}".format(z+1))
                cv2.drawChessboardCorners(image, board_sz, corners,1)
                corner_count=len(corners)
                z=z+1
            
            #if got a good image, add to matrix
            if len(corners)==board_n:
                step=successes*board_n
                k=step
                for j in range(board_n):
                    image_pnts[k][0]=corners[j][0]
                    image_pnts[k][1]=corners[j][0]
                    obj_pnts[k][0]=float(j)/float(board_w)
                    obj_pnts[k][1]=float(j)%float(board_w)
                    obj_pnts[k][2]=0.0
                    k=k+1
                points_count[successes][0]=board_n
                successes=successes+1
                time.sleep(2)
                print("-------------------------------\n")
            cv2.imshow("Test Frame", image)
            cv2.waitKey(33)
        print("Checking is fine, all matrices are created")
        cv2.destroyWindow("Test Frame")

        #Assigning new matrices according to view count
        obj_pnts2 = np.array(successes*board_n, 3, cv2.CV_32FC1)
        img_pnts2 = np.array(successes*board_n, 2, cv2.CV_32FC1)
        pnt_counts2 = np.array(successes,1,cv2.CV_32SC1)

        #transfer points to matrices
        for i in range(success*board_n):
            img_pnts2[i][0]=image_pnts[i][0]
            img_pnts2[i][1]=image_pnts[i][1]
            obj_pnts2[i][0]=obj_pnts[i][0]
            obj_pnts2[i][1]=obj_pnts[i][1]
            obj_pnts2[i][2]=obj_pnts[i][2]
        
        for i in range(successes):
            pnt_counts2[i][0]=points_count[i][0]
        
        intrinsic_matrix[0][0]=1.0
        intrinsic_matrix[1][1]=1.0

        rcv=np.array(n_boards,3, cv2.CV_64FC1)
        tcv=np.array(n_boards,3, cv2.CV_64FC1)

        print("Checking Camera Calibration..............")
        cv2.calibrateCamera(obj_pnts2, img_pnts2, pnt_counts2, cv2.getSize(image), intrinsic_matrix, distortion_coefficient, rcv, tcv,0)

        print("Checking Camera Calibration..............OK")

        #storing results in xml files
        cv2.cv.save("Intrinsic.xml", intrinsic_matrix)
        cv2.cv.save("Distortion.xml", distortion_coefficient)

        #loading from xml files
        intrinsic = cv2.cv.Load("Intrinsic.xml")
        distortion = cv2.cv.Load("Distortion.xml")
        print("loaded all distortion parameters")




        ret, frame= cap.read()
        image = cv2.iplimage(frame)
        gray = cv2.cvtColor(bpixel accuracy on those corners
        cornersframe, cv2.COLOR_BGR2GRAY)
        mapx= cv2.CreateImage(cv2.GetSize(image), cv2.IPL_DEPTH_32F, 1)
        mapy= cv2.CreateImage(cv2.GetSize(image), cv2.IPL_DEPTH_32F, 1)

        #Initializes rectification matrices
        cv2.InitUndistortMap(intrinsic, distortion, mapx, mapy)
        t=cv2.CloneImage(image)

        #Rectify Image
        cv2.Remap(t, image, mapx, mapy)

        #Get the chessboard on the plane
        cv2.NamedWindow("Chessboard")
        
        if(~found):
        out.write(frame)
            print("Couldn't aquire chessboard on"+argv[5]+"only found"+corner+"of"+board_n+"corners")

       (found, corners) = cv2.FindChessboardCorners(image, board_sz, cv2.CV_CALIB_CB_ADAPTIVE_THRESH| cv2.CV_CALIB_CB_FILTER_QUADS)
        #Get Subpixel accuracy on those corners
        corners = cv2.FindCornerSubPix(gray_image, corners, (11,11), (-1,-1),(cv2.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER,30,0.1) )
            
        #Get the image and object points
        #We will choose chessboard object points as (radius,center)
        #(0,0),(board_w-1,0), (0,board_h-1), (board_w-1,board_h-1)
        objPts[0]=[0,0]
        objPts[1]=[board_w-1, 0]
        objPts[2]=[0,board_h-1]
        objPts[3]=[board_w-1, board_h-1]

        imgPts[0]=corners[0]
        imgPts[1]=corners[board_w-1]
        imgPts[2]=corners[(board_h-1)*board_w]
        imgPts[3]=corners[(board_h-1)*board_w+board_w-1]

        #Draw the points in order : B,G,R,Y
        cv2.circle(image, imgPts[0], 9, CV_RGB(0,0,255)) #img, center, radius, color, thickness=1, lineType=8, shift=0
        cv2.circle(image, imgPts[1], 9, CV_RGB(0,255,0))
        cv2.circle(image, imgPts[2], 9, CV_RGB(255,0,0))
        cv2.circle(image, imgPts[3], 9, CV_RGB(255,255,0))

        #Draw the found chessboard

        cv2.DrawChessboardCorners(image, board_sz, coreners, 1)
        cv2.ShowImage("Chessboard", image)

        #Find the Homography
        H=cv2.CreateMat(3,3,cv.CV_32F)
        cv2.getPerspectiveTransform(objPts, imgPts, H)

        #Let the user adjust the z height of the view
        Z=25
        key =0
        bird_image=cv2.CloneImage(image)
        cv2.NamedWindow("BirdCrs_Eye")

        out.write(frame)
        #Loop to allow user to play with height
        #escape key stops
        while(key < 27):
            #CV_MAT_ELEM(*H, float, 2, 2) = Z;
            cv2.warpPerspective(image, bird_image, H, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS)
            cv2.ShowImage("Birds_Eye", bird_image)
            cv2.WaitKey()
            if(key == 'u'):
                Z+=0.5
            if(key == 'd'):
                Z-=0.5
        cv2.Save("H.xml",)

    cap.release()
    cv2.destroyAllWindows()




