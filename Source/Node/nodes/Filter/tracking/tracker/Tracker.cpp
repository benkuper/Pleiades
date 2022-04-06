void ImageBlobTracker::track(std::vector<Object> & detectedObjects, int _current_frame_nb)
{

    /* #region CLEANING */ 



    //remove objects set to willLeave in last frame
    for (size_t i = 0; i<m_pTrackedObjects.size(); i++) {
        if(m_pTrackedObjects.at(i).state == WillLeave) {
            m_pTrackedObjects.erase(m_pTrackedObjects.begin() + i);
        }
    }
    /* #endregion */

    // will store the indexes of the objects in the detectedObjects vector for which we did find a matching object in the m_pTrackedObjects vector
    // we need this vector to be here so that we can use it at the end of this function to perform the ghosting step
    // 
    //  Example content:
    //    
    //  m_pTrackedObjectIndex        detectedObjectsIndex
    //          [0]             ->          3           // the detectedObject[3] matches the m_pTrackedObject[0]
    //          [1]             ->          1           // the detectedObject[1] matches the m_pTrackedObject[1]
    //          [2]             ->          -1          // no match found for m_pTrackedObject[2]
    //          [3]             ->          2           // the detectedObject[2] matches the m_pTrackedObject[3]
    std::vector <int> assignement;
    
    // will store the indexes of new objects in the detectedObjects vector that must be added to the m_pTrackedObjects vector at the end of this function
    // we need this vector to be here so we can add the new objects at the very end of this function and not mess with m_pTrackedObjects
    //
    // Example : newObjects = {3,6,7} -> we have 3 new objects that where not tracked (e.g. they appear during this frame), we must add detectedObjects[3], detectedObjects[6], detectedObjects[7] to the m_pTrackedObjects 
    std::vector<int> newObjects;

    //there are blobs detected
    if (!detectedObjects.empty()) {

        /* #region INITIALIZE WHEN THERE ARE NO TRACKED OBJECTS */ 
        //if there is no objects already tracked, initialize them all from the detected blobs
        if (m_pTrackedObjects.empty()) {
            for (size_t i = 0; i<detectedObjects.size(); i++) {
                m_pTrackedObjects.push_back(detectedObjects.at(i));
                //also fill assignments vector 
                assignement.push_back(i);
            }
            /* #endregion */
        // else (ie there are some objects tracked)
        } else {
            /* #region TRACKING */

            // We'll need to store the sum of all the velocity vectors
            cv::Point2f totalMotion = Point2f(0.f,0.f);

            // First we need to find the best match between old and new objects with the Hungarian Algorithm

            // search window
            double min_dist = 50;
            // larger search window for ghost
            double min_dist_ghost = 100;

            // matrix of the distance between each old object to each new object
            // dist(i,j) = distance between the i^th old object and the j^th new object
            std::vector <std::vector<double>> dist;
            dist.resize(m_pTrackedObjects.size());

            for(int i=0; i<m_pTrackedObjects.size(); i++) {

                dist.at(i).resize(detectedObjects.size());
                Object & obj = m_pTrackedObjects.at(i);

                auto timeDiff = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - obj.lastTime).count() / 1000;
                Point2f predict = obj.centroid + timeDiff * obj.velocity; // Dumb prediction

                for(int j=0; j<detectedObjects.size();j++) {
                    
                    Object & other = detectedObjects.at(j);
                    Point2f centroid = other.centroid;

                    float d = sqrt(pow(centroid.x - predict.x,2) + pow(centroid.y - predict.y,2)); // distance between the predicted position and the new measure

                    // Gives huge distance on purpose if it doesn't meet certain conditions
                    if((!(obj.state == Ghost) && d > min_dist) || d > 4*sqrt(obj.boundingRect.size.width*obj.boundingRect.size.height)) dist.at(i).at(j) = 10000;
                    else if ((obj.state == Ghost) && d > min_dist_ghost) dist.at(i).at(j) = 1000;
                    else dist.at(i).at(j) = d;
                }
            }

            // we also want to give an advantage to old objects, so we don't simply take the distance as cost function

            // Parameters
            float mini = 0.2; // the smaller mini is the more old objects are advantaged
            float mu = 0;
            float sigma = 2; // the higher sigma is the slower the coef decreases at the begining (function of the age)
            float amplitude = 25; 

            // Cost matrix
            std::vector <std::vector<double>> cost; 
            cost.resize(m_pTrackedObjects.size());

            for (int i=0; i<m_pTrackedObjects.size(); i++) {
                cost.at(i).resize(detectedObjects.size());
                for (int j=0; j<detectedObjects.size(); j++) {
                    Object & obj = m_pTrackedObjects.at(i);
                    int age = obj.age;
                    // ghost have a bigger scope, the cost function gives them an advantage
                    if (obj.state == Ghost) age += 20;
                    float coef = 2*amplitude*(1-mini) / (1 + exp((age-mu)/sigma)) + mini;
                    cost.at(i).at(j) = coef*(dist.at(i).at(j) + 10); // add a fixed distance 
                }
            }

            // assignement(i) = j means the j^th new object matches with the i^th old object
            // assignement(i) = -1 means no assignement found
            // if j is not assigned it means it is a new object
            // std::vector <int> assignement;
            hungarian.Solve(cost,assignement);

            assert(cost.size() == assignement.size() && "Cost and assignement must have the same size");

            if(!m_blobSettings.isLidarVideoPipe){
                // In case two points far from each other were assigned even with the dissuasive cost given to this assignement (happens if m_pObject.size = rawObject.size = 1)
                for (int i = 0; i<assignement.size(); i++) {
                    if (assignement.at(i)!=-1 && dist.at(i).at(assignement.at(i))>100) assignement.at(i) = -1;
                }
            }

            // Get shared ownership of the tracking parameters
            std::shared_ptr<double const> smoothCoefficient;
            std::shared_ptr<double const> velocityThreshold;
            std::shared_ptr<double const> predictionCoefficient;
            std::shared_ptr<double const> maxRatio;
            std::shared_ptr<double const> orientationSmooth;
            
            /**
             * @brief Allows to know if we already logged an error 
             * 
             * This avoids to flood the logs for errors that can appear in a loop
             */
            static bool logError{true};

            try{
                smoothCoefficient = std::shared_ptr<double const>{m_blobSettings.m_pSmoothCoefficient};
                velocityThreshold = std::shared_ptr<double const>{m_blobSettings.m_pVelocityThreshold};
                predictionCoefficient = std::shared_ptr<double const>{m_blobSettings.m_pPredictionCoef};
                maxRatio = std::shared_ptr<double const>{m_blobSettings.m_pRatioLocking};
                orientationSmooth = std::shared_ptr<double const>{m_blobSettings.m_pOrientationSmoothing};
                logError = true;
            } catch (const std::exception& e){
                // Set default values if we could not get valid values
                *const_cast<double*>(smoothCoefficient.get()) = 0.0; 
                *const_cast<double*>(velocityThreshold.get()) = 0.0;
                *const_cast<double*>(predictionCoefficient.get()) = 0.0;
                *const_cast<double*>(maxRatio.get()) = 1.5;
                *const_cast<double*>(orientationSmooth.get()) = 0.0;

                if(logError) {
                    LogManager::log("ImageBlobTracker","Could not get access to Tracker settings",err);
                    logError = false;
                }
            }

            /* #endregion TRACKING */

            // Process each point
            for (size_t i = 0; i<m_pTrackedObjects.size(); i++) {

                Object & obj = m_pTrackedObjects.at(i);

                int best_id = assignement.at(i); 

                //Update old object with new info
                if (best_id != -1) {

                    /* #region UPDATE VALUES FOR MATCHED OBJECTS */ 
                    Object & other = detectedObjects.at(static_cast<ulong>(best_id)); // NEW ONE      

                    // unghost obj if needed
                    if (obj.state == Ghost) {
                            obj.ghostAge = 0;
                            obj.ghostAgeSeconds = 0;
                            obj.state = Updated;
                    }   

                    // Save the centroid for later
                    obj.pushCentroid(other.centroid);
                    obj.pushCentroidMeters(other.centroidInMeters);

                    //Check if the angle has not flipped side
                    if(Point2f(cos(obj.boundingRect.angle*3.1415/180),sin(obj.boundingRect.angle*3.1415/180)).dot(Point2f(cos(other.boundingRect.angle*3.1415/180),sin(other.boundingRect.angle*3.1415/180))) < 0){
                        other.boundingRect = cv::RotatedRect(other.boundingRect.center,other.boundingRect.size,other.boundingRect.angle+180);
                    }

                    //////////////////// ONE EURO FILTER //////////////////////

                    // Time in sec interval to compute velocity values in px/s and m/s
                    float Te = (std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - obj.lastTime).count())/1000;
                    obj.pushTime(Te);
                    obj.ageSeconds += Te;
                    // std::cout << Te << ',' << obj.age << std::endl;

                    ///// DATA IN METERS
                    // In order to be sure not to interfere with the smoothing for data in pixels, 
                    // be sure that every variables here are independant from the rest of the smoothing

                    // Smoothing Parameters for data in meters
                    float beta_m = 0.5;
                    if(m_blobSettings.isLidarVideoPipe) beta_m = 1.2;
                    float min_cutoff_m = 0.1;
                    float dcutoff_m = 1;

                    // Velocity data
                    Point2f dx_m;
                    Point2f edx_m;

                    float maxVelocityMetersPerSec = 10; // Velocity Threshold
                    if(m_blobSettings.isLidarVideoPipe) maxVelocityMetersPerSec = 50;

                    float cutoff_m; // function of edx_m with parameters min_cutoff_m and beta_m

                    // Centroid in meters
                    Point2f newCentroidInMeters;
                    Point2f newVelocityMetersPerSec;
                    Point2f oldPositionInMeters = obj.centroidInMeters;
                    Point2f oldVelocityMetersPerSec = obj.velocityMetersPerSec;

                    // dx_m = (other.centroidInMeters - oldPositionInMeters) / Te;
                    dx_m = obj.getVelocityAverageMetersPerSec();
                    edx_m = filter(dx_m, oldVelocityMetersPerSec, alpha(Te,dcutoff_m));

                    cutoff_m = min_cutoff_m + beta_m * cv::norm(edx_m);
                    newCentroidInMeters = filter(other.centroidInMeters,oldPositionInMeters,alpha(Te,cutoff_m));

                    newVelocityMetersPerSec = edx_m;

                    // to prevent jitter or invalid position 
                    if (cv::norm(edx_m) > maxVelocityMetersPerSec || !other.isPositionInMetersValid) {
                        newVelocityMetersPerSec = oldVelocityMetersPerSec;
                        newCentroidInMeters = oldPositionInMeters;
                    }

                    ///// DATA IN METERS
                    // data in meters will be usefull here for a more "universal" smoothing (ie not depending on the height of the camera)

                    //Compute new data
                    Point2f newCentroid;
                    Point2f newVelocity;
                    cv::RotatedRect newBoundingRect;
                    float newVelocityAngle;
                    cv::RotatedRect newBoundingRectVelocity;

                    //First save old position and old velocity
                    Point2f oldPosition = obj.centroid;
                    cv::Point2f oldVelocity = obj.velocity;

                    // One Euro Filter's parameters
                    // if high speed lag is a problem, increase β; if slow speed jitter is a problem, decrease fcmin.
                    float min_cutoff; // minimun cutoff
                    double beta; // cutoff slope
                    float dcutoff; // derivative cutoff
                    float cutoff; // cutoff
                    
                    cv::Point2f dx;
                    cv::Point2f edx; 
                    
                    min_cutoff = 0.01;
                    beta = 0.2;
                    if(m_blobSettings.isLidarVideoPipe) beta = 0.5;
                    dcutoff = 1;
                
                    // Even if the centorid if computed in pixels, for a more "universal" smoothig, the cutoff frequency is computed from values in m/s
                    dx = obj.getVelocityAverage();
                    edx = filter(dx, oldVelocity,alpha(Te,dcutoff));

                    // Centroid 
                    cutoff = min_cutoff + beta * 50 * cv::norm(edx_m);
                    newCentroid = filter(other.centroid,oldPosition,alpha(Te,cutoff));

                    // Velocity
                    newVelocity = edx;

                    // to prevent from having infinite velocity...
                    float maxVelocity = 300; 
                    if(m_blobSettings.isLidarVideoPipe) maxVelocity = 1500;
                    if (cv::norm(edx) > maxVelocity) {
                        newVelocity = oldVelocity;
                        newCentroid = oldPosition; // prevents jitter 
                    }

                    totalMotion += newVelocity; 
                    // Prediction
                    float e = sqrt((newCentroid.x-oldPosition.x)*(newCentroid.x-oldPosition.x) + (newCentroid.y-oldPosition.y)*(newCentroid.y-oldPosition.y));
                    if(e>1) newCentroid += 10*(*predictionCoefficient)*Te*newVelocity;

                    // Bounding box 
                    double bbx; 
                    double bby; 
                    double bbw; 
                    double bbh; 
                    double bba; 

                    double ratio; // ratio between width and height for better smoothing on bounding box orientation
                    // Computing bounding box center (position and velocity)
                    cv::Point2f oldCenterVelocity = obj.boundingRectVelocity.center;
                    cv::Point2f newCenterVelocity; 

                    cv::Point2f oldCenter = obj.boundingRect.center;
                    cv::Point2f newCenter; // bounding box center

                    // dx = (other.boundingRect.center - oldCenter) / Te; 
                    // edx = filter(dx,oldCenterVelocity,alpha(Te,dcutoff));

                    // cutoff = min_cutoff + beta*cv::norm(edx);
                    // instead we juste take same values as the centroid 

                    newCenter = filter(other.boundingRect.center,oldCenter,alpha(Te,cutoff));

                    newCenterVelocity = edx;
                    // Prediction
                    if(e>1) newCenter += 10*(*predictionCoefficient)*Te*newCenterVelocity;
                    bbx = newCenter.x;
                    bby = newCenter.y;

                    // Computing bounding box dimension (values and "velocity")
                    float beta_bb = 0.5;
                    float min_cutoff_bb = 0.001;
                    cv::Point2f oldDimVelocity = obj.boundingRectVelocity.size;  
                    cv::Point2f newDimVelocity; 

                    cv::Point2f oldDim = obj.boundingRect.size;
                    cv::Point2f newDim;
                    
                    dx = (cv::Point2f(other.boundingRect.size) - oldDim) / Te; 
                    edx = filter(dx,oldDimVelocity,alpha(Te,dcutoff));

                    cutoff = min_cutoff_bb + beta_bb*cv::norm(edx);
                    newDim = filter(cv::Point2f(other.boundingRect.size),oldDim,alpha(Te,cutoff));

                    bbh = newDim.x;
                    bbw = newDim.y;

                    newDimVelocity = edx;

                    // add extra smoothing
                    bbw = (*smoothCoefficient)*oldDim.x+ (1-(*smoothCoefficient))*other.boundingRect.size.width;
                    bbh = (*smoothCoefficient)*oldDim.y + (1-(*smoothCoefficient))*other.boundingRect.size.height;

                    ratio = max(bbh,bbw)/min(bbh,bbw);

                    // Computing bounding box angle (value and velocity)
                    float beta_a = 0.05;
                    float min_cutoff_a = min_cutoff/10;
                    float dxa; 
                    float edxa;

                    float oldAngle = obj.boundingRect.angle;
                    float newAngle; 
                    float oldVelocityAngle = obj.boundingRectVelocity.angle; 

                    dxa = (other.boundingRect.angle - oldAngle) / Te;
                    edxa =  alpha(Te,dcutoff)*dxa + (1-alpha(Te,dcutoff))*oldVelocityAngle; 

                    cutoff = min_cutoff_a + beta_a*std::abs(edxa);
                    bba = alpha(Te,cutoff)*other.boundingRect.angle + (1-alpha(Te,cutoff))*oldAngle;
                    newAngle = bba;

                    // Check angle has not flipped
                    if (abs(floor(oldAngle-bba))==90 || abs(floor(oldAngle-bba))==180) {
                        bba = oldAngle;
                    }
                    
                    // add smoothing when velocity is low 
                    if (obj.getVelocityAverageNormMetersPerSec()<(*velocityThreshold)) {
                        bba = lerp(oldAngle, newAngle, obj.currentAlpha);
                    }


                    if(ratio < (*maxRatio)) {
                        bba = lerp(newAngle, oldAngle, (*orientationSmooth));
                    }

                    if (ratio < (*maxRatio) && obj.getVelocityAverageNormMetersPerSec()<(*velocityThreshold)) {
                        // bba = lerp(oldAngle, newAngle, 0.8);
                        bba = oldAngle;
                    }
                    // bba = lerp(oldAngle, other.boundingRect.angle, obj.currentAlpha); // ONLY EXP SMOOTHING CURRENT ALPHA
                    // bba = other.boundingRect.angle; /// NO SMOOTHING TEST 
                    // bba = lerp(oldAngle, other.boundingRect.angle, 0.2); // ONLY EXP SMOOTHING CONSTANT ALPHA
                    // bba = alpha(Te,cutoff)*other.boundingRect.angle + (1-alpha(Te,cutoff))*oldAngle; // ONLY ONE EURO

                    newVelocityAngle = edxa;

                    newBoundingRect = cv::RotatedRect(cv::Point2f(bbx,bby),cv::Size2f(bbw,bbh),bba); // Half one euro filter / half exponential smooth                    

                    newBoundingRectVelocity = cv::RotatedRect(newCenterVelocity,newDimVelocity,newVelocityAngle);

                    // newBoundingRect = cv::RotatedRect(cv::Point2f(bbx,bby),cv::Size2f(bbw,bbh),bba);

                    // // Then add the calculated float values to the map, we'll use them for the computations in the next loop
                    // // It prevents points from freezing far from the actual blob because of the int approximation
                    // m_pNextBoundingBoxFloats->erase(obj.pid);
                    // (*m_pNextBoundingBoxFloats)[obj.pid].push_back(bbx);
                    // (*m_pNextBoundingBoxFloats)[obj.pid].push_back(bby);
                    // (*m_pNextBoundingBoxFloats)[obj.pid].push_back(bbw);
                    // (*m_pNextBoundingBoxFloats)[obj.pid].push_back(bbh);
                    // (*m_pNextBoundingBoxFloats)[obj.pid].push_back(bba);

                    // //build new bounding rect with computed values
                    
                    // Orientation 

                    beta_a = 0.0005;
                    float newOrientation;
                    float oldOrientation = obj.orientation;  
                    float newOrientationVelocity; 
                    float oldOrientationVelocity = obj.orientationVelocity;  

                    // cv::Point2f avgVelocity = obj.getVelocityAverage()/Te; // Approximation, Te has to be almost constant
                    cv::Point2f avgVelocity = obj.getVelocityAverage(); /////// TEST AVEC LE TIME BUFFER
                    float avgAcceleration = obj.getAverageAccelerationMetersPerSec2(); // dérivée de la norme de la vitesse 
                    // std::cout << avgAcceleration << std::endl;

                    if (obj.getVelocityAverageNormMetersPerSec()<(*velocityThreshold) / 2) {

                        // No movement, we take the bounding box angle with a smoothing
                        // newOrientation = lerp(obj.boundingRect.angle + obj.orientationOffset,obj.orientation,obj.currentAlpha);
                        newOrientation = lerp(obj.boundingRect.angle + obj.orientationOffset,obj.orientation,(*orientationSmooth));

                    } else if(abs(avgAcceleration)<0.5) {

                        // Rectilinear uniform motion -> orientation doesn't change
                        newOrientation = oldOrientation;

                    } else if(abs(avgAcceleration)>50) {

                        // Jitter -> orientation doesn't change
                        newOrientation = oldOrientation;

                    } else {

                        // Unspecified movement 
                        newOrientation = acos(cv::Point2f(1,0).dot(avgVelocity)/(cv::norm(avgVelocity)));
                        newOrientation *= 180/(M_PI);
                        
                        //check if the angle is correctly oriented
                        if(newVelocity.cross(cv::Point2f(1,0))>0){
                            newOrientation = 2*M_PI-newOrientation;
                        }
                        
                        dxa = (newOrientation - oldOrientation)/Te; 
                        edxa = alpha(Te,dcutoff)*dxa + (1-alpha(Te,dcutoff))*oldOrientationVelocity;

                        cutoff = min_cutoff_a + beta_a*std::abs(dxa);
                        newOrientation = alpha(Te,cutoff)*newOrientation + (1-alpha(Te,cutoff))*oldOrientation; 

                        newOrientationVelocity = edxa;
                    }

                    // update current alpha value
                    if(obj.getStandardDeviation()>(*velocityThreshold)){
                        if(!obj.isUnderThreshold){
                            obj.currentAlpha = baseAlpha;
                            if(obj.lockOrientation && avgAcceleration>(*velocityThreshold)){
                                obj.lockOrientation = false;
                            }
                            if(!obj.lockOrientation && avgAcceleration>(*velocityThreshold)*(-2.0)){
                                obj.orientationOffset = newOrientation - obj.boundingRect.angle; 
                            } else {
                                obj.lockOrientation = true;
                            }
                        }
                        //hysteresis
                        if(obj.isUnderThreshold && cv::norm(newVelocity)>((*velocityThreshold)*3)&&cv::norm(newCenterVelocity)>((*velocityThreshold)*3)){
                            obj.currentAlpha = baseAlpha;
                            obj.isUnderThreshold = false;
                        }
                    } else {
                        obj.isUnderThreshold = true;
                        auto now = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<float> deltaTime = now - obj.lastTime;
                        obj.currentAlpha -= alphaDecreasePerSecond * deltaTime.count();
                        obj.currentAlpha = clamp(obj.currentAlpha,minAlpha,baseAlpha);
                    }

                    //              UPDATE INFO             //

                    obj.centroid = newCentroid;
                    obj.velocity = newVelocity;
                    obj.centroidInMeters = newCentroidInMeters; // data in meters
                    obj.velocityMetersPerSec = newVelocityMetersPerSec; // data in meters
                    obj.boundingRect = newBoundingRect;
                    obj.orientation = newOrientation; 
                    obj.orientationVelocity = newOrientationVelocity;
                    obj.boundingRectVelocity = newBoundingRectVelocity;

                    obj.area = other.area;
                    obj.convexHull = other.convexHull;
                    obj.distance = other.distance;
                    obj.state = Updated;
                    obj.height = other.height;
                    obj.heightLocation = other.heightLocation;

                    /* #endregion */
                }
            }

            // When all the points have been processed, we compute the average velocity
            Point2f averageVelocity = Point2f(totalMotion.x/m_pTrackedObjects.size(), totalMotion.y/m_pTrackedObjects.size());
            //std::cout << "Average velocity : " << (int)avgVel.x << "," << (int)avgVel.y << std::endl;
            m_pScene->averageMotion = averageVelocity;
            
            /* #region ADD NEWLY DETECTED OBJECTS TO THE NEW OBJECTS LIST */ 

            // newObjects stores the id of objects that were not assigned to an old object, naive implementation

            std::vector<int> assignedObject;
            for (int k=0;k<assignement.size();k++) {
                if(assignement.at(k)!=-1) {
                    assignedObject.push_back(assignement.at(k));
                }
            }

            for (int j=0;j<detectedObjects.size();j++) {
                bool isNew = true;
                for (int i=0;i<assignedObject.size();i++) {
                    if(j==assignedObject.at(i)) {
                        isNew = false;
                    }
                }
                if (isNew) {
                    newObjects.push_back(j);
                }
            }

            // for (int k=0;k<assignement.size();k++) {
            //     std::cout << assignement.at(k) << " , ";
            // }
            // std::cout << std::endl;

            // std::cout << assignedObject.size() << "---------"; 
            // for(int i=0;i<assignement.size();i++) {
                // std::cout << assignement.at(i) << " , ";
            // }
            // std::cout << std::endl;

            /* #endregion */

            // Manage the double buffer of bounding box floats
            m_pCurrentBoundingBoxFloats->clear();
            if (m_pCurrentBoundingBoxFloats == &m_boundingBoxFloatsBuffer1){
                m_pCurrentBoundingBoxFloats = &m_boundingBoxFloatsBuffer2;
                m_pNextBoundingBoxFloats = &m_boundingBoxFloatsBuffer1;
            } else {
                m_pCurrentBoundingBoxFloats = &m_boundingBoxFloatsBuffer1;
                m_pNextBoundingBoxFloats = &m_boundingBoxFloatsBuffer2;
            }

        }

    } else {
        // No detected objects, fill the assignements vector to -1 so every point will be turned into ghost below (if they are old enough)
        for(int i=0; i < m_pTrackedObjects.size(); i++){
            assignement.push_back(-1);
        }

    }

    // here we do all the ghosting and lifetimes updates (age in seconds/frames, ghostAge etc..)
    assert(m_pTrackedObjects.size() == assignement.size() && "tracked object vector and assignment vector must have the same size");
    
    // first we turn each tracked object that has not matched a detected object in this frame into a ghost
    for(int i=0; i<m_pTrackedObjects.size(); i++){
        
        auto& obj = m_pTrackedObjects.at(i);
        
        // -1 -> no matching object found
        if(assignement.at(i) == -1){

            if(*m_blobSettings.m_pGhostingEnabled){
                // we did not find a match for this tracked object in the incoming detected objects, apply the ghosting algorithm 

                /* #region GHOSTING */
                obj.velocity = Point2f(0, 0);

                if(obj.state != Ghost){
                    // this object was not a ghost
                    if(obj.ageSeconds > minAgeForGhost){
                        // if it is old enough, we turn it into a ghost
                        obj.state = Ghost;
                        obj.ghostAge = 0;
                        obj.ghostAgeSeconds = 0;
                    } else {
                        // otherwise if it had a brief lifetime, it has good chances to be some unwanted noise
                        obj.state = WillLeave;
                    }
                } else {
                    // this object was already a ghost
                    if(obj.ghostAgeSeconds > maxGhostAge){
                        obj.state = WillLeave;
                    }
                }
                /* #endregion */
            } else {
                obj.state = WillLeave;
            }

        }

        updateAge(obj);
        obj.oid = static_cast<int>(i);

    }

    int newObjectsIndexStart = m_pTrackedObjects.size();
    
    // add the new objects at the end of the tracked object vector
    for(int i=0; i < newObjects.size(); i++){
        auto& obj = detectedObjects.at(newObjects.at(i)); 
        updateAge(obj);
        obj.oid = newObjectsIndexStart + i;
        m_pTrackedObjects.push_back(obj);
    }
}

void ImageBlobTracker::updateAge(Object& object){

    if(object.state == Ghost){
        object.ghostAge += 1;
        float Te = (std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - object.lastTime).count())/1000;
        object.ghostAgeSeconds += Te;
    }
    
    object.lastTime = std::chrono::high_resolution_clock::now();
    object.age++;
}

cv::Mat ImageBlobTracker::getMat(cv::Mat frame){
    // Visual only...
    // warning : the current_debug_frame is not valid anymore after that (which is normally not a problem since it's that last thing we do)
    //current_debug_frame.create(320,240,CV_8UC1);
    if (frame.type() == CV_16UC1) {
        // first we convert the source frame from 16 bits to 8bits for display
        frame.convertTo(frame, CV_8UC1, scale_16to8);
    }
    // Make it binary
    frame.setTo(255, frame);
    // Then copy into a color compatible mat
    cvtColor(frame, m_matTrack, COLOR_GRAY2BGR);
    // Draw the tracking data
    draw(frame);
    // Return the binary image + draw
    return m_matTrack;
}

// Draw the detection/tracking info in the buffer to display it in the webserver
void ImageBlobTracker::draw(cv::Mat frame)
{
    bool drawGhosts = false;
    double min, max;
    cv::minMaxIdx(frame, &min, &max);
    cv::Mat adjMap;
    frame.convertTo(adjMap, CV_8UC1, 255/(max-min), -min);
    /*
    cv::Mat falseColorMap;
    applyColorMap(adjMap, falseColorMap, cv::COLORMAP_JET);
    */
    // Draw two lines to form a cross indicating the position of the highest point

    //imshow("current frame", current_debug_frame);
    objectVectorMutex.lock();
    std::vector<Object> objectVectorCopy(m_pTrackedObjects);
    objectVectorMutex.unlock();

    if (!objectVectorCopy.empty()) {
        for (size_t i = 0; i<objectVectorCopy.size(); i++) {
            Object & obj = objectVectorCopy[i];

            if(!obj.isMature()) continue;   //skip drawing this object according to temporal filtering

            if(obj.state == WillLeave) continue;

            if(!drawGhosts && obj.state == Ghost) continue;

            if(obj.state == Ghost) obj.color = cv::Scalar(120,120,120);

            Point2f vertices[4];
            Point2f verticesExp[4];

            // Draw the bounding box
            obj.boundingRect.points(vertices);
            for (int i = 0; i < 4; i++){
                line(m_matTrack, vertices[i], vertices[(i+1)%4], obj.color,1);
            }

            float ratio = std::max(obj.boundingRect.size.height,obj.boundingRect.size.width) / std::min(obj.boundingRect.size.height,obj.boundingRect.size.width);
            putText(m_matTrack, std::to_string(ratio), obj.centroid + Point2f(10,10), CV_FONT_HERSHEY_PLAIN, 0.5, obj.color, 0.01);

//            //draw threshold
//            circle(m_matTrack, Point(static_cast<int>(obj.centroid.x), static_cast<int>(obj.centroid.y)), velocityThreshold, cv::Scalar(0,255,0));
//            circle(m_matTrack, Point(static_cast<int>(obj.centroid.x), static_cast<int>(obj.centroid.y)), 2*velocityThreshold, cv::Scalar(0,127,0));


//            //draw velocity
//            if(obj.velocity.x!=0 || obj.velocity.y!=0){
//               // cv::Point2f pt(obj.boundingRect.center.x+(obj.velocity.x)/cv::norm(obj.velocity)*10.0,obj.boundingRect.center.y+(obj.velocity.y)/cv::norm(obj.velocity)*10.0);
//                cv::Point2f pt(static_cast<int>(obj.centroid.x+(obj.velocity.x)),static_cast<int>(obj.centroid.y+(obj.velocity.y)));
//                line(m_matTrack,obj.centroid,pt,Scalar(255,0,255),1);
//            }

            //draw bounding box angle
            // line(m_matTrack,obj.boundingRect.center,obj.boundingRect.center+cv::Point2f(cos(obj.boundingRect.angle*3.1415/180.0),sin(obj.boundingRect.angle*3.1415/180.0))*10.0,Scalar(0,255,0),2);
            // std::cout << obj.boundingRect.angle << std::endl; 

            //draw orientation 
            line(m_matTrack,obj.centroid, obj.centroid + cv::Point2f((cos(obj.orientation*3.1415/180.0)),(sin(obj.orientation*3.1415/180.0)))*(obj.boundingRect.size.height/2),obj.color,1);
            

            // draws a line with an angle of zero
            // line(m_matTrack,Point2i(m_matTrack.cols/2,m_matTrack.rows/2),Point2f(m_matTrack.cols/2,m_matTrack.rows/2)+cv::Point2f(cos(0.7),sin(0.7))*100.0,Scalar(255,0,255),2);

            //draw object centroid
            circle(m_matTrack, Point(static_cast<int>(obj.centroid.x), static_cast<int>(obj.centroid.y)), 2, obj.color,-1);


//            //debug for lines appearing (might be a race condition)
//            cv::Point2f hl = obj.heightLocation;
//            cv::Point2f c = obj.centroid;
//            double dist = cv::norm(hl-c);
//            if(dist > 100.0){
//                std::cout << "Centroid pos: " << c << std::endl;
//                std::cout << "Height location: " << hl << std::endl;
//                std::cout << "Distance:  " << dist << std::endl;
//                std::cout << "Is distance the same ? " << cv::norm(obj.heightLocation-obj.centroid) << std::endl;
//                std::cout << "=================================================" << std::endl;
//            }

            // Draw two lines to form a cross indicating the position of the highest point
            int crossSize = 2;
            line(m_matTrack, Point(static_cast<int>(obj.heightLocation.x-crossSize), static_cast<int>(obj.heightLocation.y)), Point(static_cast<int>(obj.heightLocation.x+crossSize), static_cast<int>(obj.heightLocation.y)), obj.color);
            line(m_matTrack, Point(static_cast<int>(obj.heightLocation.x), static_cast<int>(obj.heightLocation.y-crossSize)), Point(static_cast<int>(obj.heightLocation.x), static_cast<int>(obj.heightLocation.y+crossSize)), obj.color);

            /*
            rectangle(falseColorMap, obj.boundingRect, Scalar(255,255,255));
            line(falseColorMap, Point(obj.heightLocation.x-crossSize, obj.heightLocation.y), Point(obj.heightLocation.x+crossSize, obj.heightLocation.y), Scalar( 255,255,255 ));
            line(falseColorMap, Point(obj.heightLocation.x, obj.heightLocation.y-crossSize), Point(obj.heightLocation.x, obj.heightLocation.y+crossSize), Scalar( 255,255, 255 ));
            */
        } 

    }
    //imshow("Projected tracked",falseColorMap);
}

std::string ImageBlobTracker::getTrackedObjectsJson(float width, float height, float offsetX, float offsetY, bool invertX, bool invertY)
{
    // to use only while tracking is in pixels
    float screenWidth = 640;
    float screenHeight = 480;

    std::string jsonStr = "";

    for(int i = 0; i < m_pTrackedObjects.size(); i++) {
        if(i>0){
             jsonStr += ", ";
        }

        float x = m_pTrackedObjects.at(i).centroid.x / (screenWidth/width) - width/2.0 + offsetX;
        if(invertX){
            x = (screenWidth - m_pTrackedObjects.at(i).centroid.x) / (screenWidth/width) - width/2.0 + offsetX;
        }
        float y = m_pTrackedObjects.at(i).centroid.y / (screenHeight/height) - height/2.0 + offsetY;
        if(invertY){
            y = (screenHeight - m_pTrackedObjects.at(i).centroid.y) / (screenHeight/height) - height/2.0 + offsetY;
        }

        jsonStr += "{\"posX\":" + std::to_string(x)
                +  ", \"posY\":" + std::to_string(y)
                +  ", \"size\":" + std::to_string(std::sqrt(m_pTrackedObjects.at(i).area) / ((screenWidth+screenHeight)/2.0) / (1+1/((width+height)/2)) )
                +  "}";
    }

    return jsonStr;
}

cv::Point2f ImageBlobTracker::filter(const cv::Point2f x, const cv::Point2f xhatprev,const float alpha) {
    return (alpha*x + (1-alpha)*xhatprev); 
}

float ImageBlobTracker::alpha(const float Te,const float cutoff) {
    float tau = 1 / (2*M_PI*cutoff);
    return (1/(1+tau/Te));
}