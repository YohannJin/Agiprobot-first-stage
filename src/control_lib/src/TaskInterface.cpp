#include "control_lib/TaskInterface.h"


int Spiralsuche::plan(TaskInterface* taskInterface)
{   
    if (taskInterface->radius_searchfield_ < distance_trajectory_)
    {
        ROS_ERROR_STREAM("ERROR: planning not possible radius_searchfield_ must be greater than distance_trajectory_");
        return 1;
    }
    else if (approximation_increment_ > distance_steps_)
    {
        ROS_ERROR_STREAM("ERROR: planning not accurate enougth distance_steps_ must be greater than approximation_increment_");
        return 1;
    }

    double phi1 = 0;
    double phi2 = 0;
    double a = distance_trajectory_/(2 * PI);
    std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

    std::vector<std::vector<double>> path;
    std::vector<double> xPoints, zPoints;
    int i = 0;
    while(true)
    {
        double phi2Constraint = pow(tan(-phi2 + phi1) * phi1 * a, 2) + pow(a * phi2 - cos(-phi2 + phi1) * phi1 * a, 2);

        if(phi2Constraint > pow(distance_steps_, 2) || (i == 0 && phi2Constraint > pow(distance_trajectory_, 2)))
        {
            i++;

            double x1 = a * phi1 * cos(phi1);
            double z1 = a * phi1 * sin(phi1);

            double x2 = a * phi2 * cos(phi2);
            double z2 = a * phi2 * sin(phi2);

            double xComponent = x2 - x1;
            double zComponent = z2 - z1;
            
            xPoints.push_back(x2);
            zPoints.push_back(z2);

            std::vector<double> transformationVector = {xComponent, 0, zComponent, 0, 0, 0};
            std::vector<double> waypoint = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, transformationVector);
            currentPose = waypoint;
            path.push_back(waypoint);

            phi1 = phi2;

            if(taskInterface->radius_searchfield_ <= sqrt(pow(x2, 2) + pow(z2, 2)))
            {
                break;
            }
        }
        phi2 = phi2 + approximation_increment_;
    }
    path_ = path;

    xPoints.insert(xPoints.begin(), 0);
    zPoints.insert(zPoints.begin(), 0);
    std::vector<std::pair<std::string, std::vector<double>>> vals = {{"xValue", xPoints}, {"zValue", zPoints}};
    write_csv("/home/nucuser/repos/wbk-effector-group/urcontroller2/plot/sp_search.csv", vals);

    return 0;
}

int Spiralsuche::execute(double distanceTrajectory, TaskInterface* taskInterface)
{   
    taskInterface->Robot_Task->rtde_control->zeroFtSensor();
    
    auto startPlanningTime = std::chrono::system_clock::now();
    distance_trajectory_ = distanceTrajectory;
    if(plan(taskInterface) == 1) return 1;
    auto endPlanningTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSecondsPlan;
    elapsedSecondsPlan = endPlanningTime-startPlanningTime;
    taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

    taskInterface->Drill_Task->triggerProg(FASTEN);
    taskInterface->Drill_Task->triggerStart();
    auto startScrewingTime = std::chrono::system_clock::now();


    std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
    taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
    std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

    for(int h = 0; h < path_.size(); h++)
    {   
        //ROS_INFO_STREAM((double)h/path_.size() << " %");

        std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

        std::vector<double> TCPForce = refTCPForce;
        bool targetNewWp = false;
        do
        {   
            double angle, velocity, acceleration;
            double deceleration = 5;
            acceleration = taskInterface->acceleration_;
            if(TCPForce.at(1) <= taskInterface->strength_max_ && TCPForce.at(1) >= taskInterface->strength_min_)
            {
                angle = 0;
                velocity = taskInterface->velocity_;
            }
            else
            {   
                refTCPForce.at(1) = TCPForce.at(1);
                
                double strengthRange = 4;
                double maxAngle = 45;
                if(TCPForce.at(1) > taskInterface->strength_max_)
                {   
                    double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
                    angle = percentage * maxAngle;
                    if(angle > maxAngle)
                    {
                        angle = 89.999;
                        velocity = 0.004;
                        acceleration = 0.002;
                    }
                }
                else if (TCPForce.at(1) < taskInterface->strength_min_)
                {   
                    double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
                    angle = -percentage * maxAngle;
                    if(angle < maxAngle)
                    {
                        angle = -89.999;
                        velocity = 0.004;
                        acceleration = 0.002;
                    }
                }

                velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
                if(velocity > 0.004)
                    velocity = 0.004;
            }

            std::vector<double> pathVecCurrentPose = taskInterface->Robot_Task->TransformBaseToToolFrame({path_[h][0], path_[h][1], path_[h][2]});
            double pathVecCurrentPoseMag = sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
            double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

            std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
            pathVecCurrentPose.at(1) = wpOffset;
            std::vector<double> newWp = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
            taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

            bool adjustAngle = false;
            int progress;
            do
            {
                bool isWrenchSafetylimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 15;
                double currentDepth = taskInterface->Robot_Task->calcDepth(contactPose);
                bool isMaxDepthReached =  abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
                if(taskInterface->Drill_Task->isDrillReady())
                {   
                    taskInterface->Robot_Task->rtde_control->stopL(50);

                    auto endScrewingTime = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsedSecondsScrewing;
                    elapsedSecondsScrewing = endScrewingTime-startScrewingTime;
                    if((double)elapsedSecondsScrewing.count() > 2)
                    {
                        taskInterface->Drill_Task->triggerProg(FASTEN);
                        taskInterface->Drill_Task->triggerStart();
                        startScrewingTime = std::chrono::system_clock::now();
                        break;
                    }
                    else
                    {   
                        //taskInterface->unscrew();

                        ROS_INFO_STREAM("SUCCESS: srcew found");
                        return 0;
                    }
                }
                else if(isMaxDepthReached || isWrenchSafetylimit)
                {   
                    taskInterface->Robot_Task->rtde_control->stopL();
                    if(isMaxDepthReached)
                    {
                        ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
                        return 2;
                    }
                    else if(isWrenchSafetylimit)
                    {
                        ROS_ERROR_STREAM("ERROR: Wrench to high, aborting ...!");
                        return 3;
                    }
                }


                progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

                TCPForce = taskInterface->Robot_Task->getToolFrameForce();

                double strengthMid = taskInterface->strength_min_ + (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;
                if(angle > 0)
                {
                    adjustAngle = TCPForce.at(1) < strengthMid;
                    if(TCPForce.at(1) > refTCPForce.at(1))
                    {
                        adjustAngle = true;
                        refTCPForce.at(1) = TCPForce.at(1);
                    }
                }
                else if(angle < 0)
                {
                    adjustAngle = TCPForce.at(1) > strengthMid;
                    if(TCPForce.at(1) < refTCPForce.at(1))
                    {
                        adjustAngle = true;
                        refTCPForce.at(1) = TCPForce.at(1);
                    }
                }
                else
                    adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ || TCPForce.at(1) < taskInterface->strength_min_;
            } while (progress >= 0 && !adjustAngle);
            taskInterface->Robot_Task->rtde_control->stopL(deceleration);

            if (progress < 0)
                targetNewWp = true;
        } while (!targetNewWp);
    }
    taskInterface->Robot_Task->rtde_control->stopL();

    ROS_ERROR_STREAM("ERROR: screw not found!");
    return 1;
}

int Spiralsuche::executeQuadrantSearch(double distanceTrajectory, double offset, TaskInterface* taskInterface)
{
    double sumPlaningTime = 0;
    int returnValue = -1;
    int counter = 0;

    std::vector<double> startPos = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

    do
    {   
        returnValue = -1;
        taskInterface->Robot_Task->rtde_control->zeroFtSensor();

        auto startPlanningTime = std::chrono::system_clock::now();
        distance_trajectory_ = distanceTrajectory;
        if(plan(taskInterface) == 1) return 1;
        auto endPlanningTime = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSecondsPlan;
        elapsedSecondsPlan = endPlanningTime-startPlanningTime;
        sumPlaningTime = sumPlaningTime + (double)elapsedSecondsPlan.count();

        taskInterface->Drill_Task->triggerProg(FASTEN);
        taskInterface->Drill_Task->triggerStart();
        auto startScrewingTime = std::chrono::system_clock::now();


        std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
        taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
        std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

        for(int h = 0; h < path_.size(); h++) 
        {   
            //ROS_INFO_STREAM((double)h/path_.size() << " %");

            std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

            std::vector<double> TCPForce = refTCPForce;
            bool targetNewWp = false;
            do
            {   
                double angle, velocity, acceleration;
                double deceleration = 5;
                acceleration = taskInterface->acceleration_;
                if(TCPForce.at(1) <= taskInterface->strength_max_ && TCPForce.at(1) >= taskInterface->strength_min_)
                {
                    angle = 0;
                    velocity = taskInterface->velocity_;
                }
                else
                {   
                    refTCPForce.at(1) = TCPForce.at(1);
                
                    double strengthRange = 4;
                    double maxAngle = 45;
                    if(TCPForce.at(1) > taskInterface->strength_max_)
                    {   
                        double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
                        angle = percentage * maxAngle;
                        if(angle > maxAngle)
                        {
                            angle = 89.999;
                            velocity = 0.004;
                            acceleration = 0.002;
                        }
                    }
                    else if (TCPForce.at(1) < taskInterface->strength_min_)
                    {   
                        double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
                        angle = -percentage * maxAngle;
                        if(angle < maxAngle)
                        {
                            angle = -89.999;
                            velocity = 0.004;
                            acceleration = 0.002;
                        }
                    }

                    velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
                    if(velocity > 0.004)
                        velocity = 0.004;
                }
                
                std::vector<double> pathVecCurrentPose = taskInterface->Robot_Task->TransformBaseToToolFrame({path_[h][0], path_[h][1], path_[h][2]});
                double pathVecCurrentPoseMag = sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
                double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

                std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
                pathVecCurrentPose.at(1) = wpOffset;
                std::vector<double> newWp = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
                taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

                bool adjustAngle = false;
                int progress;
                do
                {
                    double isWrenchSafetylimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 15;
                    double currentDepth = taskInterface->Robot_Task->calcDepth(contactPose);
                    bool isMaxDepthReached =  abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
                    if(taskInterface->Drill_Task->isDrillReady())
                    {   
                        taskInterface->Robot_Task->rtde_control->stopL(50);

                        auto endScrewingTime = std::chrono::system_clock::now();
                        std::chrono::duration<double> elapsedSecondsScrewing;
                        elapsedSecondsScrewing = endScrewingTime-startScrewingTime;
                        if((double)elapsedSecondsScrewing.count() > 2)
                        {
                            taskInterface->Drill_Task->triggerProg(FASTEN);
                            taskInterface->Drill_Task->triggerStart();
                            startScrewingTime = std::chrono::system_clock::now();
                            break;
                        }
                        else
                        {   
                            //taskInterface->unscrew();

                            ROS_INFO_STREAM("SUCCESS: srcew found");
                            returnValue = 0;

                            h = path_.size();
                            targetNewWp = true;
                            break;
                        }
                    }
                    else if(isMaxDepthReached || isWrenchSafetylimit)
                    {
                        if(isMaxDepthReached)
                        {
                            ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
                            returnValue = 2;
                        }
                        else if(isWrenchSafetylimit)
                        {
                            ROS_ERROR_STREAM("ERROR: Wrench to high, aborting ...!");
                            returnValue = 3;
                        }
                        h = path_.size();
                        targetNewWp = true;
                        break;
                    }


                    progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

                    TCPForce = taskInterface->Robot_Task->getToolFrameForce();

                    double strengthMid = taskInterface->strength_min_ + (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;
                    if(angle > 0)
                    {
                        adjustAngle = TCPForce.at(1) < strengthMid;
                        if(TCPForce.at(1) > refTCPForce.at(1))
                        {
                            adjustAngle = true;
                            refTCPForce.at(1) = TCPForce.at(1);
                        }
                    }
                    else if(angle < 0)
                    {
                        adjustAngle = TCPForce.at(1) > strengthMid;
                        if(TCPForce.at(1) < refTCPForce.at(1))
                        {
                            adjustAngle = true;
                            refTCPForce.at(1) = TCPForce.at(1);
                        }
                    }
                    else
                        adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ || TCPForce.at(1) < taskInterface->strength_min_;
                } while (progress >= 0 && !adjustAngle);
                taskInterface->Robot_Task->rtde_control->stopL(deceleration);

                if (progress < 0)
                    targetNewWp = true;
            } while (!targetNewWp);
        }
        taskInterface->Robot_Task->rtde_control->stopL();

        if(returnValue == -1)
        {
            ROS_ERROR_STREAM("ERROR: screw not found!");
            returnValue = 1;
        }

        if(returnValue != 0 && counter < 4)
        {
            std::vector<double> upPos = taskInterface->Robot_Task->rtde_control->poseTrans(taskInterface->Robot_Task->rtde_receive->getActualTCPPose(), {0, 0.02, 0, 0, 0, 0});
            taskInterface->Robot_Task->gotoTP(upPos, false, 0.05, 0.02);
            taskInterface->Robot_Task->gotoTP(startPos, false, 0.05, 0.02);

            if(counter == 0)
            {
                std::vector<double> currentPose = taskInterface->Robot_Task->rtde_control->poseTrans(startPos, {offset, 0, 0, 0, 0, 0});
                taskInterface->Robot_Task->gotoTP(currentPose, false, 0.05, 0.02);
            }
            else if(counter == 1)
            {
                std::vector<double> currentPose = taskInterface->Robot_Task->rtde_control->poseTrans(startPos, {0, 0, offset, 0, 0, 0});
                taskInterface->Robot_Task->gotoTP(currentPose, false, 0.05, 0.02);
            }
            else if(counter == 2)
            {
                std::vector<double> currentPose = taskInterface->Robot_Task->rtde_control->poseTrans(startPos, {-offset, 0, 0, 0, 0, 0});
                taskInterface->Robot_Task->gotoTP(currentPose, false, 0.05, 0.02);
            }
            else if(counter == 3)
            {
                std::vector<double> currentPose = taskInterface->Robot_Task->rtde_control->poseTrans(startPos, {0, 0, -offset, 0, 0, 0});
                taskInterface->Robot_Task->gotoTP(currentPose, false, 0.05, 0.02);
            }
        }

        //ROS_INFO_STREAM(counter + 1 << " / " << 4 << " executed");
        
        counter++;
    } while(counter < 5 && returnValue != 0);
    
    taskInterface->planningTime_.push_back(sumPlaningTime);
    return returnValue;
}

int Spiralsuche::executeWithLateralControl(double distanceTrajectory, TaskInterface* taskInterface)
{
    taskInterface->Robot_Task->rtde_control->zeroFtSensor();
    
    auto startPlanningTime = std::chrono::system_clock::now();
    distance_trajectory_ = distanceTrajectory;
    distance_steps_ = distanceTrajectory;
    if(plan(taskInterface) == 1) return 1;
    auto endPlanningTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSecondsPlan;
    elapsedSecondsPlan = endPlanningTime-startPlanningTime;
    taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

    taskInterface->Drill_Task->triggerProg(FASTEN);
    taskInterface->Drill_Task->triggerStart();
    auto startScrewingTime = std::chrono::system_clock::now();


    std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
    taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
    std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();


    bool movedUp = false;
    int blockForcemode = 0;
    for(int h = 0; h < path_.size(); h++)
    {   
        //ROS_INFO_STREAM((double)h/path_.size() << " %");

        std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

        std::vector<double> TCPForce = refTCPForce;
        bool targetNewWp = false;
        int progress;
        do
        {   
            double angle, velocity, acceleration;
            acceleration = taskInterface->acceleration_;
            double deceleration = 10;
            double strengthMid = taskInterface->strength_min_ + (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;

            if(movedUp)
            {
                angle = 0;
                velocity = taskInterface->velocity_;
                acceleration = taskInterface->acceleration_;
            }
            else
            {   
                if(TCPForce.at(1) <= taskInterface->strength_max_ && TCPForce.at(1) >= taskInterface->strength_min_)
                {
                    angle = 0;
                    velocity = taskInterface->velocity_;
                    acceleration = taskInterface->acceleration_;
                }
                else
                {   
                    refTCPForce.at(1) = TCPForce.at(1);
                    
                    double strengthRange = 4;
                    double maxAngle = 45;
                    if(TCPForce.at(1) > taskInterface->strength_max_)
                    {   
                        double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
                        angle = percentage * maxAngle;
                        if(angle > maxAngle)
                        {
                            angle = 89.999;
                            velocity = 0.004;
                            acceleration = 0.002;
                        }
                    }
                    else if (TCPForce.at(1) < taskInterface->strength_min_)
                    {   
                        double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
                        angle = -percentage * maxAngle;
                        if(angle < maxAngle)
                        {
                            angle = -89.999;
                            velocity = 0.004;
                            acceleration = 0.002;
                        }

                        blockForcemode = 0;
                    }

                    velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
                    if(velocity > 0.004)
                        velocity = 0.004;
                }
            }
            //ROS_ERROR_STREAM("Angel: " << angle << "\nAcceleration: " << acceleration << "\nVelocity: " << velocity << "\nDeceleration: " << deceleration << "\nForce: " << TCPForce.at(1));
            
            std::vector<double> currentPose;
            std::vector<double> newWp;
            bool isWrenchLimit;

            auto startPauseTimer = std::chrono::system_clock::now();
            double elapsedSecondsPause = 0;
            do
            {   
                std::vector<double> pathVecCurrentPose = taskInterface->Robot_Task->TransformBaseToToolFrame({path_[h][0], path_[h][1], path_[h][2]});
                double pathVecCurrentPoseMag = sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
                double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

                currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
                pathVecCurrentPose.at(1) = wpOffset;
                newWp = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
                taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

                
                progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();
                targetNewWp = false;
                while(movedUp && (progress >= 0 && elapsedSecondsPause < 1))
                {   
                    auto endPauseTimer = std::chrono::system_clock::now();
                    std::chrono::duration<double> ChronoElapsedSeconds = endPauseTimer - startPauseTimer;
                    elapsedSecondsPause = (double)ChronoElapsedSeconds.count();

                    progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();
                    if(progress < 0 && h < path_.size() - 1)
                    {
                        h++;
                        targetNewWp = true;
                    }

                    TCPForce = taskInterface->Robot_Task->getToolFrameForce();
                    isWrenchLimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 5;
                    if(isWrenchLimit)
                    {   
                        targetNewWp = false;
                        break;
                    }
                }
                if(movedUp)
                {
                    taskInterface->Robot_Task->rtde_control->stopL(deceleration);
                }
            } while(targetNewWp);
            movedUp = false;

            bool adjustAngle = false;
            do
            {   
                bool checkWhileForcemode = false;
                std::chrono::system_clock::time_point startCheckLateralTimer;
                do
                {   
                    TCPForce = taskInterface->Robot_Task->getToolFrameForce();
                    isWrenchLimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 5;
                    //ROS_INFO_STREAM(isWrenchLimit);
                    //ROS_INFO_STREAM(abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))));
                    if(isWrenchLimit && !movedUp && blockForcemode == 0)
                    {   
                        taskInterface->Robot_Task->rtde_control->stopL();
                        currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
                        taskInterface->Robot_Task->startForcemode(WRENCH_DOWN, 8);
                        startCheckLateralTimer = std::chrono::system_clock::now();
                        checkWhileForcemode = true;
                        blockForcemode++;
                    }
                    
                    do
                    {   
                        if(taskInterface->Drill_Task->isDrillReady())
                        {   
                            taskInterface->Robot_Task->rtde_control->stopL(50);

                            auto endScrewingTime = std::chrono::system_clock::now();
                            std::chrono::duration<double> elapsedSecondsScrewing;
                            elapsedSecondsScrewing = endScrewingTime-startScrewingTime;
                            if((double)elapsedSecondsScrewing.count() > 10)
                            {
                                taskInterface->Drill_Task->triggerProg(FASTEN);
                                taskInterface->Drill_Task->triggerStart();

                                startScrewingTime = std::chrono::system_clock::now();
                                if(!checkWhileForcemode)
                                {
                                    taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);
                                }
                            }
                            else
                            {   
                                taskInterface->Robot_Task->rtde_control->stopL();
                                taskInterface->Robot_Task->rtde_control->forceModeStop();
                                //taskInterface->unscrew();

                                ROS_INFO_STREAM("SUCCESS: srcew found");
                                return 0;
                            }
                        }

                        auto endCheckLateralTimer = std::chrono::system_clock::now();
                        std::chrono::duration<double> elapsedSecondsChecking = endCheckLateralTimer - startCheckLateralTimer;
                        if(!taskInterface->Drill_Task->isDrillReady() && checkWhileForcemode && (double)elapsedSecondsChecking.count() >= 1)
                        {   
                            checkWhileForcemode = false;
                            taskInterface->Robot_Task->rtde_control->forceModeStop();       ROS_ERROR_STREAM("stopFM");
                            taskInterface->Robot_Task->rtde_control->moveL(currentPose, 0.05, 0.02, false);
                            taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);
                        }
                    } while(checkWhileForcemode);

                    if(isWrenchLimit && !movedUp)
                    {   
                        taskInterface->Robot_Task->rtde_control->stopL();
                            
                        currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
                        std::vector<double> upPos = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, {0, 0.05, 0, 0, 0, 0});
                        taskInterface->Robot_Task->rtde_control->moveL(upPos, 0.01, 0.015, true);
                        movedUp = true;
                        adjustAngle = true;
                    }
                    if(!isWrenchLimit && movedUp)
                    {   
                        taskInterface->Robot_Task->rtde_control->stopL();
                        currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
                        std::vector<double> upPos = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, {0, 0.001, 0, 0, 0, 0});
                        taskInterface->Robot_Task->rtde_control->moveL(upPos, 0.01, 0.015, false);
                    }
        
                } while (isWrenchLimit);

                double currentDepth = taskInterface->Robot_Task->calcDepth(contactPose);
                bool isMaxDepthReached =  abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
                if(isMaxDepthReached)
                {   
                    taskInterface->Robot_Task->rtde_control->stopL();
                    ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
                    return 2;
                }


                progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();
                
                if(!movedUp)
                {
                    if(angle > 0)
                    {
                        adjustAngle = TCPForce.at(1) < strengthMid;
                        if(TCPForce.at(1) > refTCPForce.at(1))
                        {
                            adjustAngle = true;
                            refTCPForce.at(1) = TCPForce.at(1);
                        }
                    }
                    else if(angle < 0)
                    {
                        adjustAngle = TCPForce.at(1) > strengthMid;
                        if(TCPForce.at(1) < refTCPForce.at(1))
                        {
                            adjustAngle = true;
                            refTCPForce.at(1) = TCPForce.at(1);
                        }
                    }
                    else
                        adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ || TCPForce.at(1) < taskInterface->strength_min_;
                }
            } while (progress >= 0 && !adjustAngle);
            taskInterface->Robot_Task->rtde_control->stopL(deceleration);

            if (progress < 0)
                targetNewWp = true;
        } while (!targetNewWp);
    }
    taskInterface->Robot_Task->rtde_control->stopL();

    ROS_ERROR_STREAM("ERROR: screw not found!");
    return 1;
}


int SukzessiveApproximationBrian::plan(TaskInterface* taskInterface)
{
    if (taskInterface->radius_searchfield_ < distance_steps_)
    {
        ROS_ERROR_STREAM("ERROR: planning not possible radius_searchfield_ must be greater than distance_steps_");
        return 1;
    }
    

    int nextLocation = 0;
    std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
    double xComponent, zComponent;
    double sumXComponent = 0;
    double sumZComponent = 0;
    double lastXComponent = 0;
    double lastZComponent = 0;

    std::vector<std::vector<double>> path;
    std::vector<double> xPoints, zPoints;
    srand(time(NULL));
    for(int i = 0; i < number_tries_; i++)
    {   
        bool calcDirection = true;
        while(calcDirection)
        {
            nextLocation = rand() % 8 + 1;

            switch(nextLocation)
            {
                case 1: xComponent = distance_steps_;
                        zComponent = 0;
                        break;
                case 2: xComponent = distance_steps_ * sin(PI/4);
                        zComponent = distance_steps_ * cos(PI/4);
                        break;
                case 3: xComponent = 0;
                        zComponent = distance_steps_;
                        break;
                case 4: xComponent = -distance_steps_ * sin(PI/4);
                        zComponent = distance_steps_ * cos(PI/4);
                        break;
                case 5: xComponent = -distance_steps_;
                        zComponent = 0;
                        break;
                case 6: xComponent = -distance_steps_ * sin(PI/4);
                        zComponent = -distance_steps_ * cos(PI/4);
                        break;
                case 7: xComponent = 0;
                        zComponent = -distance_steps_;
                        break;
                case 8: xComponent = distance_steps_ * sin(PI/4);
                        zComponent = -distance_steps_ * cos(PI/4);
                        break;
            }

            sumXComponent = sumXComponent + xComponent;
            sumZComponent = sumZComponent + zComponent;
            double distanceFromCenter = sqrt(pow(sumXComponent, 2) + pow(sumZComponent, 2));

            if((xComponent != -lastXComponent || zComponent != -lastZComponent) && taskInterface->radius_searchfield_ >= distanceFromCenter)
                calcDirection = false;
            else
            {
                sumXComponent = sumXComponent - xComponent;
                sumZComponent = sumZComponent - zComponent;
            }
        }
        lastXComponent = xComponent;
        lastZComponent = zComponent;

        xPoints.push_back(sumXComponent);
        zPoints.push_back(sumZComponent);

        std::vector<double> transformationVector = {xComponent, 0, zComponent, 0, 0, 0};
        std::vector<double> waypoint = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, transformationVector);
        currentPose = waypoint;
        path.push_back(waypoint);
    }
    path_ = path;

    xPoints.insert(xPoints.begin(), 0);
    zPoints.insert(zPoints.begin(), 0);
    std::vector<std::pair<std::string, std::vector<double>>> vals = {{"xValue", xPoints}, {"zValue", zPoints}};
    write_csv("/home/nucuser/repos/wbk-effector-group/urcontroller2/plot/saBrian_search.csv", vals);

    return 0;
}

int SukzessiveApproximationBrian::execute(double distanceSteps, int numberTries, TaskInterface* taskInterface)
{
    taskInterface->Robot_Task->rtde_control->zeroFtSensor();
    
    distance_steps_ = distanceSteps;
    number_tries_ = numberTries;
    auto startPlanningTime = std::chrono::system_clock::now();
    if(plan(taskInterface) == 1) return 1;
    auto endPlanningTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSecondsPlan;
    elapsedSecondsPlan = endPlanningTime - startPlanningTime;
    taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

    taskInterface->Robot_Task->startForcemode(WRENCH_DOWN, taskInterface->strength_min_);
    for(int tryCount = 0; tryCount < path_.size(); tryCount++)
    {   
        taskInterface->Drill_Task->triggerProg(FASTEN);
        taskInterface->Drill_Task->triggerStart();
        auto startScrewingTime = std::chrono::system_clock::now();
        taskInterface->Robot_Task->startForcemode(WRENCH_DOWN, taskInterface->strength_min_);

        std::vector<double> TCPforce;
        do 
        {
            TCPforce = taskInterface->Robot_Task->rtde_receive->getActualTCPForce();
        } while (TCPforce[2] < taskInterface->strength_min_);

        auto start = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds;
        do
        {   
            if(taskInterface->Drill_Task->isDrillReady())
                {   
                    auto endScrewingTime = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsedSecondsScrewing;
                    elapsedSecondsScrewing = endScrewingTime-startScrewingTime;
                    if((double)elapsedSecondsScrewing.count() > 2)
                    {
                        auto startTimerRestart = std::chrono::system_clock::now();
                        taskInterface->Drill_Task->triggerProg(FASTEN);
                        taskInterface->Drill_Task->triggerStart();
                        auto endTimerRestart = std::chrono::system_clock::now();
                        start = start + (endTimerRestart - startTimerRestart);
                        startScrewingTime = std::chrono::system_clock::now();
                    }
                    else
                    {   
                        taskInterface->Robot_Task->rtde_control->forceModeStop();
                        //taskInterface->unscrew();

                        ROS_INFO_STREAM("SUCCESS: srcew found");
                        return 0;
                    }
                }
            auto end = std::chrono::system_clock::now();
            elapsed_seconds = end-start;
        } while (elapsed_seconds.count() <= contact_time_);
        
        taskInterface->Robot_Task->rtde_control->forceModeStop();
        
        taskInterface->Robot_Task->gotoTP(path_.at(tryCount), false, taskInterface->velocity_, taskInterface->acceleration_);
        taskInterface->Robot_Task->gotoTP(path_.at(tryCount), false, taskInterface->velocity_, taskInterface->acceleration_);
    }

    ROS_ERROR_STREAM("ERROR: screw not found!");
    return 1;
}


int SukzessiveApproximationNave::plan(TaskInterface* taskInterface)
{
    if (taskInterface->radius_searchfield_ < distance_steps_)
    {
        ROS_ERROR_STREAM("ERROR: planning not possible radius_searchfield_ must be greater than distance_steps_");
        return 1;
    }
    

    int nextLocation = 0;
    std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
    double xComponent, zComponent;
    double sumXComponent = 0;
    double sumZComponent = 0;
    double lastXComponent = 0;
    double lastZComponent = 0;

    std::vector<std::vector<double>> path;
    std::vector<double> xPoints, zPoints;
    srand(time(NULL));
    for(int i = 0; i < number_tries_; i++)
    {
        bool calcDirection = true;
        while(calcDirection)
        {
            nextLocation = rand() % 8 + 1;

            switch(nextLocation)
            {
                case 1: xComponent = distance_steps_;
                        zComponent = 0;
                        break;
                case 2: xComponent = distance_steps_ * sin(PI/4);
                        zComponent = distance_steps_ * cos(PI/4);
                        break;
                case 3: xComponent = 0;
                        zComponent = distance_steps_;
                        break;
                case 4: xComponent = -distance_steps_ * sin(PI/4);
                        zComponent = distance_steps_ * cos(PI/4);
                        break;
                case 5: xComponent = -distance_steps_;
                        zComponent = 0;
                        break;
                case 6: xComponent = -distance_steps_ * sin(PI/4);
                        zComponent = -distance_steps_ * cos(PI/4);
                        break;
                case 7: xComponent = 0;
                        zComponent = -distance_steps_;
                        break;
                case 8: xComponent = distance_steps_ * sin(PI/4);
                        zComponent = -distance_steps_ * cos(PI/4);
                        break;
            }

            sumXComponent = sumXComponent + xComponent;
            sumZComponent = sumZComponent + zComponent;
            double distanceFromCenter = sqrt(pow(sumXComponent, 2) + pow(sumZComponent, 2));

            if((xComponent != -lastXComponent || zComponent != -lastZComponent) && taskInterface->radius_searchfield_ >= distanceFromCenter)
                calcDirection = false;
            else
            {
                sumXComponent = sumXComponent - xComponent;
                sumZComponent = sumZComponent - zComponent;
            }
        }
        lastXComponent = xComponent;
        lastZComponent = zComponent;

        xPoints.push_back(sumXComponent);
        zPoints.push_back(sumZComponent);

        std::vector<double> transformationVector = {xComponent, 0, zComponent, 0, 0, 0};
        std::vector<double> waypoint = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, transformationVector);
        currentPose = waypoint;
        path.push_back(waypoint);
    }
    path_ = path;

    xPoints.insert(xPoints.begin(), 0);
    zPoints.insert(zPoints.begin(), 0);
    std::vector<std::pair<std::string, std::vector<double>>> vals = {{"xValue", xPoints}, {"zValue", zPoints}};
    write_csv("/home/nucuser/repos/wbk-effector-group/urcontroller2/plot/saNave_search.csv", vals);

    return 0;
}

int SukzessiveApproximationNave::execute(double distanceSteps, int numberTries, TaskInterface* taskInterface)
{
    taskInterface->Robot_Task->rtde_control->zeroFtSensor();
    
    auto startPlanningTime = std::chrono::system_clock::now();
    number_tries_ = numberTries;
    distance_steps_ = distanceSteps;
    if(plan(taskInterface) == 1) return 1;
    auto endPlanningTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSecondsPlan;
    elapsedSecondsPlan = endPlanningTime-startPlanningTime;
    taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

    taskInterface->Drill_Task->triggerProg(FASTEN);
    taskInterface->Drill_Task->triggerStart();
    auto startScrewingTime = std::chrono::system_clock::now();


    std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
    taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
    std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

    for(int h = 0; h < path_.size(); h++)
    {   
        //ROS_INFO_STREAM((double)h/path_.size() << " %");

        std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

        std::vector<double> TCPForce = refTCPForce;
        bool targetNewWp = false;
        do
        {   
            double angle, velocity, acceleration;
            double deceleration = 5;
            acceleration = taskInterface->acceleration_;
            if(TCPForce.at(1) <= taskInterface->strength_max_ && TCPForce.at(1) >= taskInterface->strength_min_)
            {
                angle = 0;
                velocity = taskInterface->velocity_;
            }
            else
            {   
                refTCPForce.at(1) = TCPForce.at(1);
                
                double strengthRange = 4;
                double maxAngle = 45;
                if(TCPForce.at(1) > taskInterface->strength_max_)
                {   
                    double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
                    angle = percentage * maxAngle;
                    if(angle > maxAngle)
                    {
                        angle = 89.999;
                        velocity = 0.004;
                        acceleration = 0.002;
                    }
                }
                else if (TCPForce.at(1) < taskInterface->strength_min_)
                {   
                    double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
                    angle = -percentage * maxAngle;
                    if(angle < maxAngle)
                    {
                        angle = -89.999;
                        velocity = 0.004;
                        acceleration = 0.002;
                    }
                }

                velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
                if(velocity > 0.004)
                    velocity = 0.004;
            }

            std::vector<double> pathVecCurrentPose = taskInterface->Robot_Task->TransformBaseToToolFrame({path_[h][0], path_[h][1], path_[h][2]});
            double pathVecCurrentPoseMag = sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
            double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

            std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
            pathVecCurrentPose.at(1) = wpOffset;
            std::vector<double> newWp = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
            taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

            bool adjustAngle = false;
            int progress;
            do
            {
                bool isWrenchSafetylimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 15;
                double currentDepth = taskInterface->Robot_Task->calcDepth(contactPose);
                bool isMaxDepthReached =  abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
                if(taskInterface->Drill_Task->isDrillReady())
                {   
                    taskInterface->Robot_Task->rtde_control->stopL(50);

                    auto endScrewingTime = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsedSecondsScrewing;
                    elapsedSecondsScrewing = endScrewingTime-startScrewingTime;
                    if((double)elapsedSecondsScrewing.count() > 2)
                    {
                        taskInterface->Drill_Task->triggerProg(FASTEN);
                        taskInterface->Drill_Task->triggerStart();
                        startScrewingTime = std::chrono::system_clock::now();
                        break;
                    }
                    else
                    {   
                        //taskInterface->unscrew();

                        ROS_INFO_STREAM("SUCCESS: srcew found");
                        return 0;
                    }
                }
                else if(isMaxDepthReached || isWrenchSafetylimit)
                {   
                    taskInterface->Robot_Task->rtde_control->stopL();
                    if(isMaxDepthReached)
                    {
                        ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
                        return 2;
                    }
                    else if(isWrenchSafetylimit)
                    {
                        ROS_ERROR_STREAM("ERROR: Wrench to high, aborting ...!");
                        return 3;
                    }
                }


                progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

                TCPForce = taskInterface->Robot_Task->getToolFrameForce();

                double strengthMid = taskInterface->strength_min_ + (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;
                if(angle > 0)
                {
                    adjustAngle = TCPForce.at(1) < strengthMid;
                    if(TCPForce.at(1) > refTCPForce.at(1))
                    {
                        adjustAngle = true;
                        refTCPForce.at(1) = TCPForce.at(1);
                    }
                }
                else if(angle < 0)
                {
                    adjustAngle = TCPForce.at(1) > strengthMid;
                    if(TCPForce.at(1) < refTCPForce.at(1))
                    {
                        adjustAngle = true;
                        refTCPForce.at(1) = TCPForce.at(1);
                    }
                }
                else
                    adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ || TCPForce.at(1) < taskInterface->strength_min_;
            } while (progress >= 0 && !adjustAngle);
            taskInterface->Robot_Task->rtde_control->stopL(deceleration);

            if (progress < 0)
                targetNewWp = true;
        } while (!targetNewWp);
    }
    taskInterface->Robot_Task->rtde_control->stopL();

    ROS_ERROR_STREAM("ERROR: screw not found!");
    return 1;
}


int LinearschwingungUmMittelpunkt::plan(TaskInterface* taskInterface)
{  
    // calc rotationIncrement for axial symetric pattern
    double IdealRotationIncrement =  2 * asin((loopwidth_ / 2.0) / taskInterface->radius_searchfield_);
    int divisor = (int)((2.0 * PI) / IdealRotationIncrement) + (4 - (int)((2.0 * PI) / IdealRotationIncrement) % 4);
    if(divisor < 1)
    {
        ROS_ERROR_STREAM("loopwidth_ to high!!");
        return 1;
    }
    double rotationIncrement = (2.0 * PI) / divisor;
    
    // initiate vectors and path points
    double distanceEndboints = loopwidth_;
    double offsetStuetzpunkt = (distanceEndboints * sqrt(2)) / 2.0;
    double stuetzpunktLength = taskInterface->radius_searchfield_ + offsetStuetzpunkt;
    double xDirectionStuetzpunkt = stuetzpunktLength;
    double zDirectionStuetzpunkt = 0;

    std::vector<double> xPoints, zPoints;

    bool approximate = true;
    int trigger2 = 0;
    double currentPhi = 0;
    for(int i = 0; i < divisor / 2; i++)
    {   
        xPoints.push_back(0);
        zPoints.push_back(0);

        int trigger = 1;
        for(int i = 0; i < 2; i++)
        {
            double xDirection = taskInterface->radius_searchfield_;
            double zDirection = 0;

            double xDirectionRotated = xDirection * cos(currentPhi) - zDirection * sin(currentPhi);
            double zDirectionRotated = xDirection * sin(currentPhi) + zDirection * cos(currentPhi);

            xPoints.push_back(xDirectionRotated);
            zPoints.push_back(zDirectionRotated);

            if(trigger == 1 && trigger2 == 0)
            {
                double xDirectionStuetzpunktRotated = xDirectionStuetzpunkt * cos(0.5 * rotationIncrement + currentPhi) 
                                                        - zDirectionStuetzpunkt * sin(0.5 * rotationIncrement + currentPhi);
                double zDirectionStuetzpunktRotated = xDirectionStuetzpunkt * sin(0.5 * rotationIncrement + currentPhi) 
                                                        + zDirectionStuetzpunkt * cos(0.5 * rotationIncrement + currentPhi);

                xPoints.push_back(xDirectionStuetzpunktRotated);
                zPoints.push_back(zDirectionStuetzpunktRotated);   
                
                currentPhi = currentPhi + rotationIncrement;
            }
            if(trigger == 1 && trigger2 == 1)
            {
                double xDirectionStuetzpunktRotated = xDirectionStuetzpunkt * cos(- 0.5 * rotationIncrement + currentPhi) 
                                                        - zDirectionStuetzpunkt * sin(- 0.5 * rotationIncrement + currentPhi);
                double zDirectionStuetzpunktRotated = xDirectionStuetzpunkt * sin(- 0.5 * rotationIncrement + currentPhi) 
                                                        + zDirectionStuetzpunkt * cos(- 0.5 * rotationIncrement + currentPhi);

                xPoints.push_back(xDirectionStuetzpunktRotated);
                zPoints.push_back(zDirectionStuetzpunktRotated);   
                
                currentPhi = currentPhi - rotationIncrement;
            }
            trigger = 0;
        }
        

        if(trigger2 == 1)
        {
            currentPhi = currentPhi + PI + 2 * rotationIncrement;
            trigger2 = 0;
        }
        else
        {
            currentPhi = currentPhi - PI;
            trigger2 = 1;
        }
    }
    xPoints.push_back(0);
    zPoints.push_back(0);

    // calculate path vectors
    std::vector<double> transformationVecInX, transformationVecInZ;
    for(int i = 0; i < xPoints.size() - 1; i++)
    {
        transformationVecInX.push_back(xPoints.at(i +1) - xPoints.at(i));
        transformationVecInZ.push_back(zPoints.at(i +1) - zPoints.at(i));
    }
    double maxBlendAtStuetzpunkt = sqrt(pow(transformationVecInX.at(1), 2)+pow(transformationVecInZ.at(1), 2));

    //calculate robotposes
    std::vector<std::vector<double>> path;
    std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
    int trigger3 = 0;
    for(int i = 0; i < transformationVecInX.size(); i++)
    {
        std::vector<double> transformationVector = {transformationVecInX.at(i), 0, transformationVecInZ.at(i), 0, 0, 0};
        std::vector<double> waypoint = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, transformationVector);
        currentPose = waypoint;

        //Note: 1;5;9;13
        double pathBlend, velocity, acceleration;
        pathBlend = 0;
        bool iIsGreaterTwo = i > 1;
        bool iIsStuetzpunkt = (i - 1) % 4 == 0;
        if((iIsGreaterTwo &&  iIsStuetzpunkt) || trigger3 == 1 || i == 1)
        {
            velocity = 10 * taskInterface->velocity_;
            acceleration = 10 * taskInterface->acceleration_;
            if(iIsGreaterTwo &&  iIsStuetzpunkt || i == 1)
            {
                pathBlend = 0.95 * maxBlendAtStuetzpunkt;
                trigger3 = 1;
            }
            else
            {
                trigger3 = 0;
            }
        }
        else
        {   
            velocity =  taskInterface->velocity_;
            acceleration = taskInterface->acceleration_;
            trigger3 = 0;
        }
        path.push_back(waypoint);
    }
    path_ = path;

    xPoints.insert(xPoints.begin(), 0);
    zPoints.insert(zPoints.begin(), 0);
    std::vector<std::pair<std::string, std::vector<double>>> vals = {{"xValue", xPoints}, {"zValue", zPoints}};
    write_csv("/home/nucuser/repos/wbk-effector-group/urcontroller2/plot/lo_search.csv", vals);
    
    return 0;
}

int LinearschwingungUmMittelpunkt::execute(double loopwidth, TaskInterface* taskInterface)
{   
    taskInterface->Robot_Task->rtde_control->zeroFtSensor();
    
    auto startPlanningTime = std::chrono::system_clock::now();
    loopwidth_ = loopwidth;
    if(plan(taskInterface) == 1) return 1;
    auto endPlanningTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSecondsPlan;
    elapsedSecondsPlan = endPlanningTime-startPlanningTime;
    taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

    taskInterface->Drill_Task->triggerProg(FASTEN);
    taskInterface->Drill_Task->triggerStart();
    auto startScrewingTime = std::chrono::system_clock::now();


    std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
    taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
    std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

    for(int h = 0; h < path_.size(); h++)
    {   
        //ROS_INFO_STREAM((double)h/path_.size() << " %");

        std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

        std::vector<double> TCPForce = refTCPForce;
        bool targetNewWp = false;
        do
        {   
            double angle, velocity, acceleration;
            double deceleration = 5;
            acceleration = taskInterface->acceleration_;
            if(TCPForce.at(1) <= taskInterface->strength_max_ && TCPForce.at(1) >= taskInterface->strength_min_)
            {
                angle = 0;
                velocity = taskInterface->velocity_;
            }
            else
            {   
                refTCPForce.at(1) = TCPForce.at(1);
                
                double strengthRange = 4;
                double maxAngle = 45;
                if(TCPForce.at(1) > taskInterface->strength_max_)
                {   
                    double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
                    angle = percentage * maxAngle;
                    if(angle > maxAngle)
                    {
                        angle = 89.999;
                        velocity = 0.004;
                        acceleration = 0.002;
                    }
                }
                else if (TCPForce.at(1) < taskInterface->strength_min_)
                {   
                    double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
                    angle = -percentage * maxAngle;
                    if(angle < maxAngle)
                    {
                        angle = -89.999;
                        velocity = 0.004;
                        acceleration = 0.002;
                    }
                }

                velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
                if(velocity > 0.004)
                    velocity = 0.0035;
            }

            std::vector<double> pathVecCurrentPose = taskInterface->Robot_Task->TransformBaseToToolFrame({path_[h][0], path_[h][1], path_[h][2]});
            double pathVecCurrentPoseMag = sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
            double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

            std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
            pathVecCurrentPose.at(1) = wpOffset;
            std::vector<double> newWp = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
            taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

            bool adjustAngle = false;
            int progress;
            do
            {
                bool isWrenchSafetylimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 15;
                double currentDepth = taskInterface->Robot_Task->calcDepth(contactPose);
                bool isMaxDepthReached =  abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
                if(taskInterface->Drill_Task->isDrillReady())
                {   
                    taskInterface->Robot_Task->rtde_control->stopL(50);

                    auto endScrewingTime = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsedSecondsScrewing;
                    elapsedSecondsScrewing = endScrewingTime-startScrewingTime;
                    if((double)elapsedSecondsScrewing.count() > 2)
                    {
                        taskInterface->Drill_Task->triggerProg(FASTEN);
                        taskInterface->Drill_Task->triggerStart();
                        startScrewingTime = std::chrono::system_clock::now();
                        break;
                    }
                    else
                    {   
                        //taskInterface->unscrew();

                        ROS_INFO_STREAM("SUCCESS: srcew found");
                        return 0;
                    }
                }
                else if(isMaxDepthReached || isWrenchSafetylimit)
                {   
                    taskInterface->Robot_Task->rtde_control->stopL();
                    if(isMaxDepthReached)
                    {
                        ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
                        return 2;
                    }
                    else if(isWrenchSafetylimit)
                    {
                        ROS_ERROR_STREAM("ERROR: Wrench to high, aborting ...!");
                        return 3;
                    }
                }


                progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

                TCPForce = taskInterface->Robot_Task->getToolFrameForce();

                double strengthMid = taskInterface->strength_min_ + (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;
                if(angle > 0)
                {
                    adjustAngle = TCPForce.at(1) < strengthMid;
                    if(TCPForce.at(1) > refTCPForce.at(1))
                    {
                        adjustAngle = true;
                        refTCPForce.at(1) = TCPForce.at(1);
                    }
                }
                else if(angle < 0)
                {
                    adjustAngle = TCPForce.at(1) > strengthMid;
                    if(TCPForce.at(1) < refTCPForce.at(1))
                    {
                        adjustAngle = true;
                        refTCPForce.at(1) = TCPForce.at(1);
                    }
                }
                else
                    adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ || TCPForce.at(1) < taskInterface->strength_min_;
            } while (progress >= 0 && !adjustAngle);
            taskInterface->Robot_Task->rtde_control->stopL(deceleration);
            //ROS_ERROR_STREAM("Force: " << TCPForce.at(1) << " / " << refTCPForce.at(1));

            if (progress < 0)
                targetNewWp = true;
        } while (!targetNewWp);
    }
    taskInterface->Robot_Task->rtde_control->stopL();

    ROS_ERROR_STREAM("ERROR: screw not found!");
    return 1;
}


int LissajousscheFiguren::plan(TaskInterface* taskInterface)
{   
    if (taskInterface->radius_searchfield_ < distance_trajectory_)
    {
        ROS_ERROR_STREAM("ERROR: planning not possible radius_searchfield_ must be greater than distance_trajectory_");
        return 1;
    }

    // calc ideal distance trajectory, for symetric pattern
    int divisor = (int)((taskInterface->radius_searchfield_ / distance_trajectory_) + 1);
    distance_trajectory_ = taskInterface->radius_searchfield_ / divisor;
    
    // calculate points
    std::vector<double> xPoints, zPoints;
    double xValue, zValue;


    zPoints.push_back(0);

    int i = 0;
    while(true)
    {
        int argument;

        if(i % 2 == 0)
        {
            argument = 1;
        }
        else
        {
            argument = -1;
        }
        
        xValue = (-distance_trajectory_ / 2 - i * distance_trajectory_) * argument;
        zValue = (-taskInterface->radius_searchfield_ + i * distance_trajectory_) * argument;
        if(abs(zValue) < abs(taskInterface->radius_searchfield_ / 2) || abs(xValue) > abs(taskInterface->radius_searchfield_ / 2))
            break;
        for(int j = 0; j < 2; j++)
        {
            zPoints.push_back(zValue);
            xPoints.push_back(xValue);
        }                 

        xValue = (distance_trajectory_ / 2 + i * distance_trajectory_) * argument;
        zValue = (taskInterface->radius_searchfield_ - i * distance_trajectory_) * argument;
        if(abs(zValue) < abs(taskInterface->radius_searchfield_ / 2) || abs(xValue) > abs(taskInterface->radius_searchfield_ / 2))
            break;
        for(int j = 0; j < 2; j++)
        {
            zPoints.push_back(zValue);
            xPoints.push_back(xValue);
        }

        xValue = (-distance_trajectory_ / 2 - i * distance_trajectory_) * argument;
        zValue = (distance_trajectory_ / 2 + i * distance_trajectory_) * argument;
        if(abs(zValue) > abs(taskInterface->radius_searchfield_ / 2) || abs(xValue) > abs(taskInterface->radius_searchfield_ / 2))
            break;
        for(int j = 0; j < 2; j++)
        {
            zPoints.push_back(zValue);
            xPoints.push_back(xValue);
        }

        xValue = (- taskInterface->radius_searchfield_ + i * distance_trajectory_) * argument;
        zValue = (-distance_trajectory_ / 2 - i * distance_trajectory_) * argument;
        if(abs(zValue) > abs(taskInterface->radius_searchfield_ / 2) || abs(xValue) < abs(taskInterface->radius_searchfield_ / 2))
            break;
        for(int j = 0; j < 2; j++)
        {
            zPoints.push_back(zValue);
            xPoints.push_back(xValue);
        }
        
        xValue = (taskInterface->radius_searchfield_ - i * distance_trajectory_) * argument;
        zValue = (distance_trajectory_ / 2 + i * distance_trajectory_) * argument;
        if(abs(zValue) > abs(taskInterface->radius_searchfield_ / 2) || abs(xValue) < abs(taskInterface->radius_searchfield_ / 2))
            break;
        for(int j = 0; j < 2; j++)
        {
            zPoints.push_back(zValue);
            xPoints.push_back(xValue);
        }

        i++;
    }

    zPoints.pop_back();

    // calculate path vectors
    std::vector<double> transformationVecInX, transformationVecInZ;
    for(int i = 0; i < xPoints.size() - 1; i++)
    {
        transformationVecInX.push_back(xPoints.at(i + 1) - xPoints.at(i));
        transformationVecInZ.push_back(zPoints.at(i + 1) - zPoints.at(i));
    }

    //calculate robotposes
    std::vector<std::vector<double>> path;
    std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
    int trigger3 = 0;
    for(int i = 0; i < transformationVecInX.size(); i++)
    {
        std::vector<double> transformationVector = {transformationVecInX.at(i), 0, transformationVecInZ.at(i), 0, 0, 0};
        std::vector<double> waypoint = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, transformationVector);
        currentPose = waypoint;
        path.push_back(waypoint);
    }
    path_ = path;

    xPoints.insert(xPoints.begin(), 0);
    zPoints.insert(zPoints.begin(), 0);
    std::vector<std::pair<std::string, std::vector<double>>> vals = {{"xValue", xPoints}, {"zValue", zPoints}};
    write_csv("/home/nucuser/repos/wbk-effector-group/urcontroller2/plot/lf_search.csv", vals);

    return 0;
}

int LissajousscheFiguren::execute(double distanceTrajectory, TaskInterface* taskInterface)
{
    taskInterface->Robot_Task->rtde_control->zeroFtSensor();
    
    auto startPlanningTime = std::chrono::system_clock::now();
    distance_trajectory_ = distanceTrajectory;
    if(plan(taskInterface) == 1) return 1;
    auto endPlanningTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSecondsPlan;
    elapsedSecondsPlan = endPlanningTime-startPlanningTime;
    taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

    taskInterface->Drill_Task->triggerProg(FASTEN);
    taskInterface->Drill_Task->triggerStart();
    auto startScrewingTime = std::chrono::system_clock::now();


    std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
    taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
    std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

    for(int h = 0; h < path_.size(); h++)
    {   
        //ROS_INFO_STREAM((double)h/path_.size() << " %");

        std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

        std::vector<double> TCPForce = refTCPForce;
        bool targetNewWp = false;
        do
        {   
            double angle, velocity, acceleration;
            double deceleration = 5;
            acceleration = taskInterface->acceleration_;
            if(TCPForce.at(1) <= taskInterface->strength_max_ && TCPForce.at(1) >= taskInterface->strength_min_)
            {
                angle = 0;
                velocity = taskInterface->velocity_;
            }
            else
            {   
                refTCPForce.at(1) = TCPForce.at(1);
                
                double strengthRange = 4;
                double maxAngle = 45;
                if(TCPForce.at(1) > taskInterface->strength_max_)
                {   
                    double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
                    angle = percentage * maxAngle;
                    if(angle > maxAngle)
                    {
                        angle = 89.999;
                        velocity = 0.004;
                        acceleration = 0.002;
                    }
                }
                else if (TCPForce.at(1) < taskInterface->strength_min_)
                {   
                    double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
                    angle = -percentage * maxAngle;
                    if(angle < maxAngle)
                    {
                        angle = -89.999;
                        velocity = 0.004;
                        acceleration = 0.002;
                    }
                }

                velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
                if(velocity > 0.004)
                    velocity = 0.004;
            }
            
            std::vector<double> pathVecCurrentPose = taskInterface->Robot_Task->TransformBaseToToolFrame({path_[h][0], path_[h][1], path_[h][2]});
            double pathVecCurrentPoseMag = sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
            double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

            std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
            pathVecCurrentPose.at(1) = wpOffset;
            std::vector<double> newWp = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
            taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

            bool adjustAngle = false;
            int progress;
            do
            {
                bool isWrenchSafetylimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 15;
                double currentDepth = taskInterface->Robot_Task->calcDepth(contactPose);
                bool isMaxDepthReached =  abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
                if(taskInterface->Drill_Task->isDrillReady())
                {   
                    taskInterface->Robot_Task->rtde_control->stopL(50);

                    auto endScrewingTime = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsedSecondsScrewing;
                    elapsedSecondsScrewing = endScrewingTime-startScrewingTime;
                    if((double)elapsedSecondsScrewing.count() > 2)
                    {
                        taskInterface->Drill_Task->triggerProg(FASTEN);
                        taskInterface->Drill_Task->triggerStart();
                        startScrewingTime = std::chrono::system_clock::now();
                        break;
                    }
                    else
                    {   
                        //taskInterface->unscrew();

                        ROS_INFO_STREAM("SUCCESS: srcew found");
                        return 0;
                    }
                }
                else if(isMaxDepthReached || isWrenchSafetylimit)
                {   
                    taskInterface->Robot_Task->rtde_control->stopL();
                    if(isMaxDepthReached)
                    {
                        ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
                        return 2;
                    }
                    else if(isWrenchSafetylimit)
                    {
                        ROS_ERROR_STREAM("ERROR: Wrench to high, aborting ...!");
                        return 3;
                    }
                }


                progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

                TCPForce = taskInterface->Robot_Task->getToolFrameForce();
                //ROS_ERROR_STREAM("Force: " << TCPForce.at(1) << " / " << refTCPForce.at(1));

                double strengthMid = taskInterface->strength_min_ + (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;
                if(angle > 0)
                {
                    adjustAngle = TCPForce.at(1) < strengthMid;
                    if(TCPForce.at(1) > refTCPForce.at(1))
                    {
                        adjustAngle = true;
                        refTCPForce.at(1) = TCPForce.at(1);
                    }
                }
                else if(angle < 0)
                {
                    adjustAngle = TCPForce.at(1) > strengthMid;
                    if(TCPForce.at(1) < refTCPForce.at(1))
                    {
                        adjustAngle = true;
                        refTCPForce.at(1) = TCPForce.at(1);
                    }
                }
                else
                    adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ || TCPForce.at(1) < taskInterface->strength_min_;
            } while (progress >= 0 && !adjustAngle);
            taskInterface->Robot_Task->rtde_control->stopL(deceleration);

            if (progress < 0)
                targetNewWp = true;
        } while (!targetNewWp);
    }
    taskInterface->Robot_Task->rtde_control->stopL();

    ROS_ERROR_STREAM("ERROR: screw not found!");
    return 1;
}



void TaskInterface::unscrew()
{
    Robot_Task->startForcemode(WRENCH_DOWN, 5);
    Drill_Task->triggerProg(LOSEN);
    //while(Drill_Task->isDrillReady()); //blocking
    Drill_Task->triggerStart();
    while(!Drill_Task->isDrillReady())
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); //blocking
    Robot_Task->rtde_control->forceModeStop();
}

bool TaskInterface::checkLateralForce(std::vector<double> moveDirection)
{
    ROS_INFO_STREAM("checkLateralForce()");

    //rotate moveDirection 90 degrees in x,z plane of tool frame, and calculate unit vector
    double xUnitCompRotated = (-moveDirection.at(2) / sqrt(pow(-moveDirection.at(2), 2) + pow(moveDirection.at(0), 2)));
    double zUnitCompRotated = (moveDirection.at(0) / sqrt(pow(-moveDirection.at(2), 2) + pow(moveDirection.at(0), 2)));

    // scale to 15 cm
    double xCompScaled = xUnitCompRotated * 0.15;
    double zCompScaled = zUnitCompRotated * 0.15;

    std::vector<double> currentPos = Robot_Task->rtde_receive->getActualTCPPose();

    Robot_Task->rtde_control->forceModeSetDamping(1);
	std::vector<double> taskFrameLeft = Robot_Task->rtde_control->poseTrans(currentPos, {xCompScaled, 0, zCompScaled, 0, 0, 0});
    std::vector<double> taskFrameRight = Robot_Task->rtde_control->poseTrans(currentPos, {-xCompScaled, 0, -zCompScaled, 0, 0, 0});
	std::vector<int> selectionVector = {0, 1, 0, 0, 0, 0};
	std::vector<double> wrenchLeft = {0, 5, 0, 0, 0, 0};
	std::vector<double> wrenchRight = {0, 5, 0, 0, 0, 0};
    int forceType = 1;
    std::vector<double> limits = {0.1, 0.005, 0.1, 0.17, 0.17, 0.17};

    bool checkLeft = false;
    bool checkRight = false;

    auto startCheckingTimer = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds;
    do
    {   
        Robot_Task->rtde_control->forceMode(taskFrameLeft, selectionVector, wrenchLeft, forceType, limits);
        std::vector<double> TCPForce = Robot_Task->getToolFrameForce();
        double reactionForce = xUnitCompRotated * TCPForce.at(0) + zUnitCompRotated * TCPForce.at(2);

        ROS_ERROR_STREAM("LEFT: " << reactionForce);
        if(reactionForce < -2)
        {   
            //currentPosLeft = Robot_Task->rtde_receive->getActualTCPPose();
            checkLeft = true;
            break;
        }

        auto endCheckingTimer = std::chrono::system_clock::now();
        elapsedSeconds = endCheckingTimer - startCheckingTimer;
    } while ((double)elapsedSeconds.count() < 0.1);
    Robot_Task->rtde_control->forceModeStop();

    Robot_Task->rtde_control->moveL(currentPos, 0.05, 0.02, false);

    startCheckingTimer = std::chrono::system_clock::now();
    do
    {   
        Robot_Task->rtde_control->forceMode(taskFrameRight, selectionVector, wrenchRight, forceType, limits);
        std::vector<double> TCPForce = Robot_Task->getToolFrameForce();
        double reactionForce = xUnitCompRotated * TCPForce.at(0) + zUnitCompRotated * TCPForce.at(2);

        ROS_ERROR_STREAM("RIGHT: " << reactionForce);
        if(reactionForce > 2)
        {
            checkRight = true;
            break;
        }


        auto endCheckingTimer = std::chrono::system_clock::now();
        elapsedSeconds = endCheckingTimer - startCheckingTimer;
    } while ((double)elapsedSeconds.count() < 0.1);
    Robot_Task->rtde_control->forceModeStop();

    Robot_Task->rtde_control->moveL(currentPos, 0.05, 0.02, false);

    ROS_INFO_STREAM(checkLeft << checkRight);
    return checkLeft && checkRight;
}
