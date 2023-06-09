/*
 *
 *  Example by Sam Siewert 
 *
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <time.h>
#include <semaphore.h>
using namespace cv;


#define HRES 640
#define VRES 480
#define SEC_TO_NSEC 1000000000
// Transform display window
char timg_window_name[] = "Edge Detector Transform";
Size img_resolution[3]={Size(640,480),Size(320,240),Size(160,120)};

int resolution_index;
int transform_index;
int lowThreshold=0;
int const max_lowThreshold = 100;
int kernel_size = 3;
int edgeThresh = 1;
int ratio = 3;
Mat canny_frame, cdst, timg_gray, timg_grad;
VideoCapture cap;
Mat frame;
pthread_barrier_t bar;
long wcet;
long deadline;
std::vector<Vec3f> circles;
std::vector<Vec4i> lines;
void Hough()
{
    Mat mat_frame(frame);
    Canny(mat_frame, canny_frame, 50, 200, 3);

    //cvtColor(canny_frame, cdst, CV_GRAY2BGR);
    //cvtColor(mat_frame, gray, CV_BGR2GRAY);

    HoughLinesP(canny_frame, lines, 1, CV_PI/180, 50, 50, 10);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line(mat_frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
    imshow( timg_window_name, frame );
    cvWaitKey(20);

}
void Elliptical()
{
    Mat mat_frame(frame);

    cvtColor(mat_frame, timg_gray, CV_BGR2GRAY);
    GaussianBlur(timg_gray, timg_gray, Size(9,9), 2, 2);

    HoughCircles(timg_gray, circles, CV_HOUGH_GRADIENT, 1, timg_gray.rows/8, 100, 50, 0, 0);

    printf("circles.size = %d\n", circles.size());

    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( mat_frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( mat_frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
    imshow( timg_window_name, frame );
    cvWaitKey(20);

}
void CannyThreshold()
{
    Mat mat_frame(frame);

    cvtColor(mat_frame, timg_gray, CV_RGB2GRAY);

    /// Reduce noise with a kernel 3x3
    blur( timg_gray, canny_frame, Size(3,3) );

    /// Canny detector
    Canny( canny_frame, canny_frame, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    timg_grad = Scalar::all(0);

    mat_frame.copyTo( timg_grad, canny_frame);

    imshow( timg_window_name, timg_grad );
    cvWaitKey(20);

}

void (*transform_funcs[3])()={CannyThreshold,Hough,Elliptical};

pthread_attr_t main_attr;
void print_scheduler(void)
{
   int schedType, scope;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
     case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
     case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n");
       break;
     case SCHED_RR:
           printf("Pthread Policy is SCHED_OTHER\n");
           break;
     default:
       printf("Pthread Policy is UNKNOWN\n");
   }

   pthread_attr_getscope(&main_attr, &scope);

   if(scope == PTHREAD_SCOPE_SYSTEM)
     printf("PTHREAD SCOPE SYSTEM\n");
   else if (scope == PTHREAD_SCOPE_PROCESS)
     printf("PTHREAD SCOPE PROCESS\n");
   else
     printf("PTHREAD SCOPE UNKNOWN\n");

}
void readFrame()
{
    Mat local_frame;
    cap.read(local_frame);
    // check if we succeeded
    if (local_frame.empty()) {
        printf("ERROR! blank frame grabbed\n");
    }

    resize(local_frame,frame,img_resolution[resolution_index],0,0,INTER_LINEAR);
}

void processFrame()
{
    transform_funcs[transform_index]();
}

sem_t sem1,sem2;
struct timespec start,stop;
unsigned long average_time=0;
unsigned long frame_no;
int complete=1; // set by logger to indicate completion, else missed deadline
void capturer(int prio)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    struct sched_param sparam;
    sparam.sched_priority=prio;
    pthread_t current_thread = pthread_self();    
    pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    pthread_setschedparam(current_thread, SCHED_FIFO,&sparam);
    
    int ret = pthread_barrier_wait(&bar);
    long sleep_ns;
    struct timespec frame_capture_ts,sleep_ts;
    while(1)
    {
        if(!complete)
        printf("MISSED DEADLINE\n");
        complete=0;
        clock_gettime(CLOCK_MONOTONIC,&start);
        deadline = start.tv_sec*SEC_TO_NSEC + start.tv_nsec + wcet;
        printf("Frame #%lu\n",frame_no++);
        printf("start at %lu\n",start.tv_sec*SEC_TO_NSEC + start.tv_nsec);
        printf("deadline with margin set %lu\n",deadline);
        readFrame();
        clock_gettime(CLOCK_MONOTONIC,&frame_capture_ts);
        sleep_ns = deadline-(frame_capture_ts.tv_sec*SEC_TO_NSEC+frame_capture_ts.tv_nsec);
        //printf("%d sleeping %lu\n",prio,sleep_ns);
        sleep_ts.tv_sec=sleep_ns/SEC_TO_NSEC;
        sleep_ts.tv_nsec=sleep_ns%SEC_TO_NSEC;

        sem_post(&sem1);
        nanosleep(&sleep_ts,0);
        
        //printf("thread %d running on %d node %d\n",prio,cpu,node);
        //sleep(1);

    }

}

void logger(int prio)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    struct sched_param sparam;
    sparam.sched_priority=prio;
    pthread_t current_thread = pthread_self();    
    pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    pthread_setschedparam(current_thread, SCHED_FIFO,&sparam);
    int ret = pthread_barrier_wait(&bar);
    long int jitter;
    
    while(1)
    {
        sem_wait(&sem2);
        clock_gettime(CLOCK_MONOTONIC,&stop);
        jitter = average_time-((stop.tv_sec-start.tv_sec)*SEC_TO_NSEC + (stop.tv_nsec-start.tv_nsec));
        complete=1;
        printf("stop at %lu. Jitter %ld\n",stop.tv_sec*SEC_TO_NSEC + stop.tv_nsec,jitter);
        
    }

}

void transformer(int prio)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    struct sched_param sparam;
    sparam.sched_priority=prio;
    pthread_t current_thread = pthread_self();    
    pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    pthread_setschedparam(current_thread, SCHED_FIFO,&sparam);
    int ret = pthread_barrier_wait(&bar);
    long sleep_ns;
    struct timespec start,stop,sleep_ts;
    while(1)
    {
        //clock_gettime(CLOCK_MONOTONIC,&start);
        //deadline = start.tv_sec*SEC_TO_NSEC + start.tv_nsec + wcet;
        //printf("deadline set %lu\n",deadline);
        sem_wait(&sem1);
        processFrame();
        //clock_gettime(CLOCK_MONOTONIC,&stop);
        //sleep_ns=deadline-(stop.tv_sec*SEC_TO_NSEC + stop.tv_nsec);
        //printf("%d sleeping %lu\n",prio,sleep_ns);
        //sleep_ts.tv_sec=sleep_ns/SEC_TO_NSEC;
        //sleep_ts.tv_nsec=sleep_ns%SEC_TO_NSEC;
        sem_post(&sem2);
        //nanosleep(&sleep_ts,0);
    }

}
void getDeadline()
{

    struct timespec start,stop;
    long sec,nsec;
    imshow( timg_window_name,0); // prevents initial lag to open image window
    for(int i=0;i<100;i++)
    {
        clock_gettime(CLOCK_MONOTONIC,&start);
        
        readFrame();
        processFrame();
        cvWaitKey(20);
        clock_gettime(CLOCK_MONOTONIC,&stop);
        sec = stop.tv_sec-start.tv_sec;
        nsec = stop.tv_nsec-start.tv_nsec;
        average_time += (sec*SEC_TO_NSEC + nsec)/100;
        printf("time taken %lu s %lu ns\n",sec,nsec);

        //sleep(1);
    }
    printf("time taken %lu ns \n",average_time);
    wcet = average_time+average_time*0.12;
    printf("wcet with margin: %lu ns\n",wcet);

}

void print_help()
{
    printf("Usage: rt_image <transform> <resolution>\n");
    printf("Transform:\n\
    Canny: 0\n \
    Hough: 1\n \
    Elliptical: 2\n");

    printf("Resolution:\n\
    640x480: 0\n\
    320x240: 1\n\
    160x120: 2\n");
    
}
int main( int argc, char** argv )
{

    int dev=0;
    char *transform;
    int resolution;
    sem_init(&sem1,0,0);
    sem_init(&sem2,0,0);
    pthread_barrier_init(&bar, NULL,3);
    if(argc != 3)
    {
        print_help();
        exit(1);
    }
    transform_index=atoi(argv[1]);
    resolution_index=atoi(argv[2]);


    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        printf("ERROR! Unable to open camera\n");
        return -1;
    }

    getDeadline();

    struct sched_param main_param;
    int rc;
    int rt_max_prio = sched_get_priority_max(SCHED_FIFO);

    rc=sched_getparam(getpid(), &main_param);
    main_param.sched_priority=rt_max_prio;

    if(rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param) < 0)
    perror("******** WARNING: sched_setscheduler");
    print_scheduler();

    std::thread *capture_thread,*transformer_thread,*logger_thread;
    capture_thread = new std::thread(capturer,rt_max_prio-1);
    transformer_thread = new std::thread(transformer,rt_max_prio-2);
    logger_thread = new std::thread(logger,rt_max_prio-3);
    capture_thread->join();
    
};
