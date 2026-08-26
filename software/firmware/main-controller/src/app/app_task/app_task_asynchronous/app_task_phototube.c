#include "../../../../inc/app/app_task/app_task_asynchronous/app_task_phototube.h"

#include "../../../../inc/app/module/module_vehicle_phototube/module_vehicle_phototube.h"

void app_task_asynchronous_phototube_run(void)
{
    module_vehicle_phototube_frame_update();
    module_vehicle_phototube_run();
}
