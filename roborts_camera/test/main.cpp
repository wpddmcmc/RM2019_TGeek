
#include "../mercure/mercure_driver.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "image_publisher");
   camera::MercureDriver driver;

   driver.ReadCamera();

    return 0;
}