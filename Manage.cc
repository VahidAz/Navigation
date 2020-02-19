#include "Manage.hh"


Manage::Manage():Debug()
{
    if ( MGDebug )
        qDebug()<<"<<<MG>>> In Constructor of Manage Class.";

    // Initialize transport
    gazebo::transport::init();

    // Create node for Communication with Laser Topic
    gazebo::transport::NodePtr nodeLaser(new gazebo::transport::Node());
    nodeLaser->Init();

    // Create node for Communication with Velocity Topic
    gazebo::transport::NodePtr nodeVel(new gazebo::transport::Node());
    nodeVel->Init();

    // Create Node For Cummunication With Info Topic
    gazebo::transport::NodePtr nodeInfo(new gazebo::transport::Node());
    nodeInfo->Init();

    // Create Node For Communication With Camera Topic
    gazebo::transport::NodePtr nodeLCam(new gazebo::transport::Node());
    nodeLCam->Init();

    gazebo::transport::NodePtr nodeRCam(new gazebo::transport::Node());
    nodeRCam->Init();

    gazebo::transport::NodePtr nodeCCam(new gazebo::transport::Node());
    nodeCCam->Init();

///***/// Each Below Subscribe that Define Last, First Call is for IT ( In Below First parseInfo Running, and After That
///***/// parseMsg and in Last parseCam

    // Listen to Gazebo Laser Topic
    subLas   = nodeLaser->Subscribe("~/pioneer3at/hokuyo/link/laser/scan", &Manage::parseLas, this);

    // Listen To Gazebo Info Topic
    subInfo  = nodeInfo->Subscribe("~/pose/info", &Manage::parseInfo, this);

    // Listen To Gazebo Left Camera Topic
    subLCam  = nodeLCam->Subscribe("~/pioneer3at/leftcamera/leftlink/camera/image", &Manage::parseLCam, this);

    // Listen To Gazebo Right Camera
    subRCam  = nodeRCam->Subscribe("~/pioneer3at/rightcamera/rightlink/camera/image", &Manage::parseRCam, this);

    // Listen To Gazebo Center Camera
    subCCam  = nodeCCam->Subscribe("~/pioneer3at/camera/link/camera/image", &Manage::parseCCam, this);

///***///

    if ( MGDebug )
        qDebug()<<"<<<MG>>> Before gazebo::transport::run()";

    // Run Transport Gazebo
    gazebo::transport::run();

    // Publish to a Gazebo Topic
    pub = nodeVel->Advertise<gazebo::msgs::Pose>("~/pioneer3at/vel_cmd");

    // Generate a pose (First Speed of Right wheels, Second Speed of Left Wheels and Third is MAX_Speed)
    // Modify SkidSteerDrivePlugin for Above Comment
    // Defualt Value For poseTemp is Zero
    poseTemp.Set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Defualt Value For Position of Robot is Zero
    posRobot.x = 0.0;
    posRobot.y = 0.0;
    posRobot.z = 0.0;

    // Defualt Value For Orientation of Robot is Zero
    rotRobot.x = 0.0;
    rotRobot.y = 0.0;
    rotRobot.z = 0.0;
    rotRobot.w = 0.0;

    // Position That LA Starts To Work
    startPos.Set(0,0,0);

    // Defualt Max Speed of Robot is 5
    maxSpeed   = 5.0;
    // Default Value For Speeds
    rightSped  = 0.0;
    leftSped   = 0.0;

    // Defualt Value For Time Is Zero
    time       = 0;

    // Current Direction of Wall Following
    curDirWall = DEFUALTNAVDIR;

    tarwall    = 0;     // Resualt Direction of LA Algorithm
    counter    = 0;     // Counter For Counting Number of Iteration For Turning and Moving
    startTime  = 0;     // Start Time of LA Algorithm
    noVic      = 0;     // Counting Number of Victim
    maxDist    = 0.0;   // Maximum Distance That Robot Moving

    // Default Value for Flags
    flagLA         = false;
    flagFuz        = true;
    flagEval       = false;

    flagRCamBlue   = false;
    flagRCamWhite  = false;
    flagRCamYellow = false;

    flagLCamBlue   = false;
    flagLCamWhite  = false;
    flagLCamYellow = false;

    flagTrnLeft    = false;
    flagTrnRight   = false;
    flagMove       = false;

    flagVictim     = false;

    flagFindVic    = false;

    // Last Position That Robot Find Victim
    lastPosVic.Set(0,0,0);

    lastPosForMove.Set(0,0,0);
    lastPosLA.Set(0,0,0);

    trAng  = 0.0;
    dirTrn = 0;
}


void Manage::start()
{
    if ( MGDebug )
        qDebug()<<"<<<MG>>> In Start Function.";

    if ( MGDebug )
        qDebug()<<"<<<MG>>> Before Wait For Connection.";

    // Wait for a subscriber to connect
    pub->WaitForConnection();

    if ( MGDebug )
        qDebug()<<"<<<MG>>> Before While in Start Function.";

    while (true)
    // Throttle Publication
        gazebo::common::Time::MSleep(100);

    // Make sure to shut everything down.
    gazebo::transport::fini();
}


void Manage::parseLas(ConstLaserScanStampedPtr &_msg)
{
    if ( MGDebug )
        qDebug()<<"<<<MG>>> In parseLas Function.";

    QList<double> speeds;
    int i;

    QString msg(_msg->ShortDebugString().c_str()), rmmsg;

///***/// Parse Laser Data

    // Cut Message Before Ranges
    rmmsg = "ranges:";
    msg   = msg.remove(0,msg.indexOf(rmmsg));

    // Cut After Ranges
    rmmsg = "intensities";
    msg   = msg.remove(msg.indexOf(rmmsg),msg.size());

    // Remove Ranges:
    rmmsg = "ranges:";
    msg   = msg.remove(rmmsg);

    // Split Msg to Seperated String
    QStringList listmsg = msg.split(QRegExp("\\s+"));

    // Clear Laser Array
    laserarray.clear();

    // Convert Laser Data from String to Double
    for (int i = 1 ; i <= (listmsg.size() - 2) ; i++ )
        laserarray.append(QString(listmsg[i]).toDouble());

///***///
    /// \ Set Laser Data For Mapping

    // Learning Automata
    if ( flagLA )
    {
        //if (sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 2 )
        //{
        if ( flagRCamBlue & flagLCamWhite & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("BLUE"),QString("WHITE"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,+1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,-1, rotRobot.z);

            flagRCamBlue  = false;
            flagLCamWhite = false;
        }
        else if ( flagRCamBlue & flagLCamYellow & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("BLUE"),QString("YELLOW"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,+1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,-1, rotRobot.z);

            flagRCamBlue   = false;
            flagLCamYellow = false;
        }
        else if ( flagRCamWhite & flagLCamBlue & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("WHITE"),QString("BLUE"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,+1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,-1, rotRobot.z);

            flagRCamWhite = false;
            flagLCamBlue  = false;
        }
        else if ( flagRCamWhite & flagLCamYellow & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("WHITE"),QString("YELLOW"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,+1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,-1, rotRobot.z);

            flagRCamWhite  = false;
            flagLCamYellow = false;
        }
        else if ( flagRCamYellow & flagLCamBlue & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("YELLOW"),QString("BLUE"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,+1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,-1, rotRobot.z);

            flagRCamYellow = false;
            flagLCamBlue   = false;
        }
        else if ( flagRCamYellow & flagLCamWhite & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("YELLOW"),QString("WHITE"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,+1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,-1, rotRobot.z);

            flagRCamYellow = false;
            flagLCamWhite  = false;
        }
        else if ( flagLCamBlue & flagRCamWhite & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("BLUE"),QString("WHITE"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,-1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,+1, rotRobot.z);

            flagLCamBlue  = false;
            flagRCamWhite = false;
        }
        else if ( flagLCamBlue & flagRCamYellow & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("BLUE"),QString("YELLOW"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,-1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,+1, rotRobot.z);

            flagLCamBlue   = false;
            flagRCamYellow = false;
        }
        else if ( flagLCamWhite & flagRCamBlue & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("WHITE"),QString("BLUE"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,-1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,+1, rotRobot.z);

            flagLCamWhite = false;
            flagRCamBlue  = false;
        }
        else if ( flagLCamWhite & flagRCamYellow & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("WHITE"),QString("YELLOW"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,-1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,+1, rotRobot.z);

            flagLCamWhite  = false;
            flagRCamYellow = false;
        }
        else if ( flagLCamYellow & flagRCamBlue & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("YELLOW"),QString("BLUE"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,-1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,+1, rotRobot.z);

            flagLCamYellow = false;
            flagRCamBlue   = false;
        }
        else if ( flagLCamYellow & flagRCamWhite & sqrt(pow(posRobot.x - lastPosLA.x , 2)+pow(posRobot.y - lastPosLA.y , 2)) >= 1)
        {
            la.setActiveActs(QString("YELLOW"),QString("WHITE"));
            tarwall = la.getLAResult();

            if ( tarwall == 1 )
                updateFlag(curDirWall,-1, rotRobot.z);
            else if ( tarwall == 2 )
                updateFlag(curDirWall,+1, rotRobot.z);

            flagLCamYellow = false;
            flagRCamWhite  = false;
        }
        else
        {
            if ( MGDebug )
                qDebug()<<"cant find LA";

            flagFuz = true;  

            flagRCamBlue = false;
            flagRCamWhite = false;
            flagRCamYellow = false;

            flagLCamBlue = false;
            flagLCamWhite = false;
            flagLCamYellow = false;
        }
    }

    // Turn To Right Or Left
    if ( flagTrnLeft || flagTrnRight )
    {
        if ( (dirTrn == +1 & crnHAng < 0 & trAng < 0 & rotRobot.z >= trAng) || (dirTrn == +1 & crnHAng < 0 & trAng >= 0 & rotRobot.z >= trAng ) )
         {
             flagTrnRight = false;
             flagTrnLeft  = false;

             flagMove     = true;
             lastPosForMove.Set(posRobot.x,posRobot.y,0);
             minDist = 10;
             for ( i = 100 ; i <= 399 ; i++ )
                 if ( laserarray[i] < minDist )
                     minDist = laserarray[i];

         }
         else if ( (dirTrn == +1 & crnHAng >= 0 & trAng > 0 & rotRobot.z >= trAng ) || ( dirTrn == +1 & crnHAng > 0 & trAng < 0 & rotRobot.z < 0 & rotRobot.z >= trAng) )
         {
            flagTrnRight = false;
            flagTrnLeft  = false;

            flagMove     = true;
            lastPosForMove.Set(posRobot.x,posRobot.y,0);
            minDist = 10;
            for ( i = 100 ; i <= 399 ; i++ )
                if ( laserarray[i] < minDist )
                    minDist = laserarray[i];
         }
         else if ( (dirTrn == -1 & crnHAng > 0 & trAng >= 0 & rotRobot.z <= trAng) || ( dirTrn == -1 & crnHAng>=0 & trAng < 0 & rotRobot.z <= trAng) )
         {
            flagTrnRight = false;
            flagTrnLeft  = false;

            flagMove     = true;
            lastPosForMove.Set(posRobot.x,posRobot.y,0);
            minDist = 10;
            for ( i = 100 ; i <= 399 ; i++ )
                if ( laserarray[i] < minDist )
                    minDist = laserarray[i];
         }
         else if ( (dirTrn == -1 & crnHAng < 0 & trAng > 0 & rotRobot.z > 0 & rotRobot.z <= trAng) || (dirTrn == -1 & crnHAng < 0 & trAng < 0 & rotRobot.z <= trAng))
         {
            flagTrnRight = false;
            flagTrnLeft  = false;

            flagMove     = true;
            lastPosForMove.Set(posRobot.x,posRobot.y,0);
            minDist = 10;
            for ( i = 100 ; i <= 399; i++ )
                if ( laserarray[i] < minDist )
                    minDist = laserarray[i];
         }
         else
         {
             if ( dirTrn == -1 ) // Turn To Right
             {
                 rightSped = -0.5;
                 leftSped  =  0.5;
             }
             else if ( dirTrn == +1 )  // Turn To Left
             {
                 rightSped =  0.5;
                 leftSped  = -0.5;
             }
         }
    }

    // Moving
    if ( flagMove )
    {
        if ( MGDebug )
            qDebug()<<"moving true";

        moving(200);
    }

    // Fuzzy
    if ( flagFuz )
    {
        if ( MGDebug )
            qDebug()<<"fuzzy true";

        if ( MGDebug )
            qDebug()<<"<<<MG>>> Flag Fuzzy Is True.";

        fz.setNavDir(curDirWall);
        fz.extractFuzIn(laserarray,posRobot.x,posRobot.y);
        speeds = fz.getFuzOutNav();

        rightSped = speeds[0];
        leftSped  = speeds[1];

        if ( MGDebug )
            qDebug()<<"<<<MG>>> Output of Fuzzy, RightSpeed, LeftSpeed: "<<speeds[0]<<speeds[1];
    }

    if ( flagEval )
    {
        if ( MGDebug )
        {
            qDebug()<<"eval true";

            qDebug()<<"time"<<time;
            qDebug()<<"starttime:"<<startTime;
            qDebug()<<"start pos: "<<startPos.x<<startPos.y;
            qDebug()<<"Cure Pos: "<<posRobot.x<<posRobot.y;
            qDebug()<<"victim No: "<<noVic;
        }

        if ( ( time - startTime > 500 )  || ( (time - startTime > 50 )  & (sqrt(pow(posRobot.x - startPos.x,2)+pow(posRobot.y - startPos.y,2)) <= 2 ) ) )  // Finish Iteration
        {
            if ( MGDebug )
                qDebug()<<"eval finished";

            la.updateProb( time - startTime, noVic, maxDist);

            noVic   = 0;
            maxDist = 0.0;

            flagEval    = false;
            flagFindVic = false;
            flagLA      = true;
            flagFuz     = false;
            stopRobot();

            lastPosLA.Set(posRobot.x,posRobot.y,0);
        }
        else if ( fz.getFuzzyKind() == "obsavoid" )
        {
            if ( MGDebug )
                qDebug()<<"eval finished, obsavoid";

            //la.updateProb( time - startTime, noVic, maxDist);

            noVic   = 0;
            maxDist = 0.0;

            flagEval    = false;
            flagFindVic = false;
            flagLA      = true;
            flagFuz     = false;
            stopRobot();

            lastPosLA.Set(posRobot.x,posRobot.y,0);
        }
        else
        {
            if ( flagVictim )
            {
                noVic++;
                flagVictim = false;

                if ( MGDebug )
                    qDebug()<<"nomber of vict: "<<noVic;
            }

            double temp = sqrt( pow(posRobot.x - startPos.x,2) + pow(posRobot.y - startPos.y,2) );

            if ( MGDebug )
                qDebug()<<"temp max: "<<temp;

            if ( temp > maxDist )
                maxDist = temp;

            if ( MGDebug )
                qDebug()<<"set max: "<<maxDist;
        }
    }
    else
    {
        flagVictim = false;
    }

    if ( MGDebug )
        qDebug()<<"<<<MG>>> Before Publish Speeds, RightSpeed, LeftSpeed, MaxSpeed: "<<rightSped<<leftSped<<maxSpeed;

    // Set Speed of Wheels
    poseTemp.Set(rightSped,leftSped, maxSpeed, 0.0, 0.0, 0.0);

    // Convert to a pose message
    gazebo::msgs::Set(&poseMsg, poseTemp);

    // Publish Message To Vel_Cmd Topic
    pub->Publish(poseMsg);
}


// In This Function We Parse Message From Info Topic
void Manage::parseInfo(ConstPosesStampedPtr &_msg)
{
    if ( MGDebug )
        qDebug()<<"<<<MG>>> In ParseInfo Function.";

    QStringList listmsg;
    QString     msg(_msg->ShortDebugString().c_str()), tempMsg, rmmsg;

    // Keep A Copy of Message For Next Parse
    tempMsg = msg;

///***/// Time Extraction

    // Cut Message Before Sec
    rmmsg = "sec:";
    msg   = msg.remove(0,msg.indexOf(rmmsg));

    // Cut After nsec
    rmmsg = "nsec";
    msg   = msg.remove(msg.indexOf(rmmsg),msg.size());

    // Remove sec:
    rmmsg = "sec: ";
    msg   = msg.remove(rmmsg);

    // Convert Timefrom String to Int
    time = msg.toInt();

    if ( MGDebug )
        qDebug()<<"<<<MG>>> Parsed Time of Simulator: "<<time;

///***///

///***/// Extract Position of Robot
    msg = tempMsg;

    // Cut Message Before First Position
    rmmsg = "position";
    msg   = msg.remove(0,msg.indexOf(rmmsg));

    // Cut After First Oriention
    rmmsg = "orientation";
    msg   = msg.remove(msg.indexOf(rmmsg),msg.size());

    // Remove Other Additional Parts
    msg   = msg.remove("position { x: ");
    msg   = msg.remove("y: ");
    msg   = msg.remove("z: ");
    msg   = msg.remove(" } ");

    // Split Msg to Seperated String
    listmsg.clear();
    listmsg = msg.split(QRegExp("\\s+"));

    // Set Position to Its Varibales
    posRobot.x = QString(listmsg[0]).toDouble();
    posRobot.y = QString(listmsg[1]).toDouble();
    posRobot.z = QString(listmsg[2]).toDouble();

    if ( MGDebug )
        qDebug()<<"<<<MG>>> Parsed Position of Robot, X: "<<posRobot.x<<" , Y: "<<posRobot.y<<" , Z: "<<posRobot.z;

///***////

///***/// Extarct Orientation of Robot
    msg = tempMsg;

    // Cut Message Before First Orientation
    rmmsg = "orientation";
    msg   = msg.remove(0,msg.indexOf(rmmsg));

    // Cut After Second Pose
    rmmsg = "pose";
    msg   = msg.remove(msg.indexOf(rmmsg,2),msg.size());

    // Remove Other Additional Parts
    msg   = msg.remove("orientation { x: ");
    msg   = msg.remove("y: ");
    msg   = msg.remove("z: ");
    msg   = msg.remove("w: ");
    msg   = msg.remove(" } } ");

    // Split Msg to Seperated String
    listmsg.clear();
    listmsg = msg.split(QRegExp("\\s+"));

    // Set Position to Its Varibales
    rotRobot.x = QString(listmsg[0]).toDouble();
    rotRobot.y = QString(listmsg[1]).toDouble();
    rotRobot.z = QString(listmsg[2]).toDouble();
    rotRobot.w = QString(listmsg[3]).toDouble();

    if ( rotRobot.w < 0 )
        rotRobot.z = rotRobot.z * -1;

    if ( MGDebug )
        qDebug()<<"<<<MG>>> Parsed Rotation of Robot (Head Angle of Head), Z: "<<rotRobot.z;

///***///
}


// In This Fucntion We Parse Image From Camera Topic
void Manage::parseRCam(ConstImageStampedPtr &_msg)
{
        if ( MGDebug )
            qDebug()<<"<<<MG>>> In parseRIGHTCam Function.";

        unsigned char *rgbData     = NULL;
        unsigned int   rgbDataSize = 0;
        int i = 0, sum = 0;

        // Convert the image data to RGB
        common::Image img;
        img.SetFromData(
                    (unsigned char *)(_msg->image().data().c_str()),
                    _msg->image().width(),
                    _msg->image().height(),
                    (common::Image::PixelFormat)(_msg->image().pixel_format()));

        img.GetRGBData(&rgbData, rgbDataSize);

        // Get the image data in a QT friendly format
        QImage image(_msg->image().width(), _msg->image().height(),
                     QImage::Format_RGB888);

        // Store the image data
        memcpy(image.bits(), rgbData, rgbDataSize);

        for ( i = 0 ; i < rgbDataSize - 3  ; i += 3 )
        {
            sum = rgbData[i] + rgbData[i+1] + rgbData[i+2];

            if ( sum == 122 ) // Blue
            {
                flagRCamBlue = true;

                break;
            }
            else if ( sum == 366 ) // White
            {
                flagRCamWhite = true;

                break;
            }
            else if ( sum == 244 ) // Yellow
            {
                flagRCamYellow = true;

                break;
            }

            flagRCamBlue   = false;
            flagRCamWhite  = false;
            flagRCamYellow = false;
        }

        delete [] rgbData;
}

void Manage::parseLCam(ConstImageStampedPtr &_msg)
{
        if ( MGDebug )
            qDebug()<<"<<<MG>>> In parseLEFTCam Function.";

        unsigned char *rgbData     = NULL;
        unsigned int   rgbDataSize = 0;
        int i = 0, sum = 0;

        // Convert the image data to RGB
        common::Image img;
        img.SetFromData(
                    (unsigned char *)(_msg->image().data().c_str()),
                    _msg->image().width(),
                    _msg->image().height(),
                    (common::Image::PixelFormat)(_msg->image().pixel_format()));

        img.GetRGBData(&rgbData, rgbDataSize);

        // Get the image data in a QT friendly format
        QImage image(_msg->image().width(), _msg->image().height(),
                     QImage::Format_RGB888);

        // Store the image data
        memcpy(image.bits(), rgbData, rgbDataSize);

        for ( i = 0 ; i < rgbDataSize - 3  ; i += 3 )
        {
            sum = rgbData[i] + rgbData[i+1] + rgbData[i+2];

            if ( sum == 122 ) // Blue
            {
                flagLCamBlue = true;

                break;
            }
            else if ( sum == 366 ) // White
            {
                flagLCamWhite = true;

                break;
            }
            else if ( sum == 244 ) // Yellow
            {
                flagLCamYellow = true;

                break;
            }

            flagLCamBlue   = false;
            flagLCamWhite  = false;
            flagLCamYellow = false;
        }

        delete [] rgbData;
}


void Manage::parseCCam(ConstImageStampedPtr &_msg)
{
    if ( flagFindVic )
    {
        if ( MGDebug )
            qDebug()<<"<<<MG>>> In parseCenterCam Function.";

        unsigned char *rgbData     = NULL;
        unsigned int   rgbDataSize = 0;
        int i = 0, sum = 0;

        // Convert the image data to RGB
        common::Image img;
        img.SetFromData(
                    (unsigned char *)(_msg->image().data().c_str()),
                    _msg->image().width(),
                    _msg->image().height(),
                    (common::Image::PixelFormat)(_msg->image().pixel_format()));

        img.GetRGBData(&rgbData, rgbDataSize);

        // Get the image data in a QT friendly format
        QImage image(_msg->image().width(), _msg->image().height(),
                     QImage::Format_RGB888);

        // Store the image data
        memcpy(image.bits(), rgbData, rgbDataSize);

        for ( i = 0 ; i < rgbDataSize - 3  ; i += 3 )
        {
            sum = rgbData[i] + rgbData[i+1] + rgbData[i+2];

            if ( sum == 0 )
            {
                if ( sqrt(pow(posRobot.x - lastPosVic.x,2)+pow(posRobot.y - lastPosVic.y,2)) >= 1.5 )  // Black
                {
                    if ( MGDebug )
                        qDebug()<<"<<<MG>>> Find Victim.";

                    flagVictim = true;

                    lastPosVic.Set(posRobot.x, posRobot.y, 0);

                    break;
                }
            }

            flagVictim = false;
        }

        delete [] rgbData;
    }
}


// Set Speed of Robot to Zero
void Manage::stopRobot()
{
    if ( MGDebug )
        qDebug()<<"<<<MG>>> In Stop Robot Function, Speeds Set to Zero.";

    rightSped = 0.0;
    leftSped  = 0.0;
}


void Manage::updateFlag(int cur, int tar, double hang)
{
    if ( cur == +1 )
    {
        if ( tar == 1 )
        {
            flagLA       = false;
            flagFuz      = true;
            flagTrnLeft  = false;
            flagTrnRight = false;

            flagEval     = true;
            flagFindVic  = true;

            startTime    = time;
            startPos.Set(posRobot.x,posRobot.y,0);

            curDirWall   = 1;
        }
        else if ( tar == -1 )
        {
            flagLA       = false;
            flagFuz      = false;
            flagTrnLeft  = true;
            flagTrnRight = false;
            flagEval     = false;
            flagFindVic  = false;

            curDirWall   = -1;

            dirTrn  = +1;
            crnHAng = rotRobot.z;
            trAng   = findTargetDegree(-1, hang);
        }
    }
    else if ( cur == -1 )
    {
        if ( tar == 1 )
        {
            flagLA       = false;
            flagFuz      = false;
            flagTrnLeft  = false;
            flagTrnRight = true;

            flagEval     = false;
            flagFindVic  = false;

            curDirWall   = 1;

            dirTrn  = -1;
            crnHAng = rotRobot.z;
            trAng   = findTargetDegree(+1, hang);
        }
        else if ( tar == -1 )
        {
            flagLA       = false;
            flagFuz      = true;
            flagTrnLeft  = false;
            flagTrnRight = false;

            flagEval     = true;
            flagFindVic  = true;

            startTime    = time;
            startPos.Set(posRobot.x,posRobot.y,0);

            curDirWall   = -1;
        }
    }
}


void Manage::moving(int ther)
{
    if ( MGDebug )
    {
        qDebug()<<minDist;
        qDebug()<<sqrt(pow(posRobot.x - lastPosForMove.x,2) + pow(posRobot.y - lastPosForMove.y,2) );
    }

    if ( sqrt(pow(posRobot.x - lastPosForMove.x,2) + pow(posRobot.y - lastPosForMove.y,2) ) < minDist )
    {
        rightSped = +1;
        leftSped  = +1;
    }
    else
    {
        flagFuz     = true;
        flagMove    = false;

        flagEval    = true;
        flagFindVic = true;

        startTime   = time;
        startPos.Set(posRobot.x,posRobot.y,0);

        stopRobot();
    }
}


double Manage::findTargetDegree(int dir, double hang)
{
    int    i = 0, ind;
    double min = 10.0, angR = -2.51, angL = 2.51, tempAng;

    if ( dir == +1 ) // Right
    {
        for ( i = 0 ; i <= 249 ; i++ )
            if ( laserarray[i] < min )
            {
                min = laserarray[i];
                minDist = min ;
                //minDist -= 0.2;
                ind = i;
            }

        for ( i = ind ; i <= 249 ; i++ )
            if ( laserarray[i] > (min * 4) )
            {
                ind = i;

                break;
            }

        for ( i = 0 ; i <= ind + 10 ; i++ )
            angR += 0.01;

        tempAng = angR / 3.14;
    }
    else if ( dir == -1 ) // Left
    {
        for ( i = 499 ; i >= 250 ; i-- )
            if ( laserarray[i] < min )
            {
                min = laserarray[i];
                minDist = min ;
                //minDist -= 0.2;
                ind = i;

                if ( MGDebug )
                    qDebug()<<min;
            }

        for ( i = ind ; i >= 250 ; i-- )
            if ( laserarray[i] > (min* 4) )
            {
                ind = i;

                if ( MGDebug )
                    qDebug()<<ind;

                break;
            }

        for ( i = 499 ; i >= ind - 30 ; i-- )
            angL -= 0.01;

        tempAng = angL / 3.14;
    }

    // If Turn Angle is Negative, It is Mean That Robot Should Turn To Right
        if ( hang < 0 & tempAng < 0 & ( hang + tempAng) < -1.0 )
                tempAng = ( 1 +  ( (hang + tempAng) + 1 ) );
        // If Turn Angle is Positive, Robot Should Turn Left
        else if ( hang > 0 & tempAng > 0 & ( hang + tempAng ) > 1.0 )
                tempAng = -1 + ( 1 - ( hang + tempAng ) );
        else
                tempAng = hang + tempAng;


        return tempAng;
}
