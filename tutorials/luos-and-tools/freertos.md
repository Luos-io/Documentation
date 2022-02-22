---
custom_edit_url: null
---

# FreeRTOS x Luos

## Use freeRTOS with Luos

If you're used to develop your embedded system with FreeRTOS, you can leverage Luos features without changing the way you design your system. Indeed, Luos fits well with FreeRTOS and this tutorial aims to help you setup your project.

This tutorial uses VSCode as IDE. You can follow each step and copy/paste the code as presented in the following but you can also clone the full project from the dedicated [repository](https://github.com/Luos-io/Examples/tree/master/Projects/l0/Button_FreeRTOS).

The project is running on [Nucleo-F072RB](https://www.st.com/en/evaluation-tools/nucleo-f072rb.html#overview) so you need one if you want to complete the tutorial.

This subject is intended for intermediate users: if you're a beginner with VSCode, PlatformIO or Luos technology, please check our [**Get Started** tutorial](https://docs.luos.io/get-started/get-started).

## First, setup a PlatformIO project

You can start from an example brought by Luos: we will use a LED example you can find [here](https://github.com/Luos-io/Examples/tree/master/Projects/l0/Led). First, create a folder and clone the repository: 

    mkdir workspace
    cd ./workspace/
    git clone https://github.com/Luos-io/Examples.git
    cd Examples/Projects/l0/Led/

For a quick reminder, you will find in this project:
- a **src/** folder containing a **main.c** whose main purpose is to call **services** and **luos** initialization functions. This folder also contains the file **stm32f0xx_it.c** in which we define interrupt handlers.
- a **lib/** folder containing services source files.

## Configure FreeRTOS for your project

Now we need to import FreeRTOS source files in our project and configure the kernel. First you can clone source files from the github repository:

    cd lib/
    git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git

Then you have to configure available options in the kernel and plug them in your project: FreeRTOS needs some resources as a timer to work properly. It also needs to handle three interrupters handlers (PendSV, SVC and Systick handlers) so you must not redefine these three handlers in **stm32f0xx_it.c**.

For more conveniance, we will download a folder with all the ressources we need to complete this tutorial:  

    cd ../../
    git clone https://github.com/Luos-io/tutorial_freertos.git
    cd Led/
    cp ../tutorial_freertos/FreeRTOSConfig.h ./inc/

If you want more information about the options displayed in this file, please check the [dedicated page](https://www.freertos.org/a00110.html). If you open this file you will notice the following section:

    #define vPortSVCHandler     SVC_Handler
    #define xPortPendSVHandler  PendSV_Handler
    #define xPortSysTickHandler SysTick_Handler

As FreeRTOS defines these handlers, you have to delete their implementation in **stm32f0xx_it.c**. Once you've done this, there is one remaining action to complete the kernel configuration: FreeRTOS needs a dedicated timer to properly works. 

The timer you choose depends of your MCU. On STMF0 we will use the **Timer 7**, which is a very simple and basic one. It's only used to call the scheduler at a fixed period (defined by **configTICK_RATE_HZ** in **FreeRTOSConfig.h**). So, let's create a new file called **stm32f0xx_hal_timebase_tim.c** in **src/** and copy the following code in it:

    #include "stm32f0xx_hal.h"
    #include "stm32f0xx_hal_tim.h"

    TIM_HandleTypeDef htim7;

    void TIM7_IRQHandler(void)
    {
        HAL_TIM_IRQHandler(&htim7);
    }

    HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
    {
        RCC_ClkInitTypeDef clkconfig;
        uint32_t uwTimclock       = 0;
        uint32_t uwPrescalerValue = 0;
        uint32_t pFLatency;
        /*Configure the TIM7 IRQ priority */
        HAL_NVIC_SetPriority(TIM7_IRQn, TickPriority, 0);

        /* Enable the TIM7 global Interrupt */
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
        /* Enable TIM7 clock */
        __HAL_RCC_TIM7_CLK_ENABLE();

        /* Get clock configuration */
        HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

        /* Compute TIM7 clock */
        uwTimclock = HAL_RCC_GetPCLK1Freq();
        /* Compute the prescaler value to have TIM7 counter clock equal to 1MHz */
        uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

        /* Initialize TIM7 */
        htim7.Instance = TIM7;

        /* Initialize TIMx peripheral as follow:
        + Period = [(TIM7CLK/1000) - 1]. to have a (1/1000) s time base.
        + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
        + ClockDivision = 0
        + Counter direction = Up
        */
        htim7.Init.Period        = (1000000U / 1000U) - 1U;
        htim7.Init.Prescaler     = uwPrescalerValue;
        htim7.Init.ClockDivision = 0;
        htim7.Init.CounterMode   = TIM_COUNTERMODE_UP;
        if (HAL_TIM_Base_Init(&htim7) == HAL_OK)
        {
            /* Start the TIM time Base generation in interrupt mode */
            return HAL_TIM_Base_Start_IT(&htim7);
        }

        /* Return function status */
        return HAL_ERROR;
    }

The function **HAL_InitTick()** is called at system startup and configures the timer to raise an interrupt flag each 1000 ms (following the period defined in **FreeRTOSConfig.h**). **TIM7_IRQHANDLER()** is the dedicated interrupt handler, it calls a HAL routine which increments the **uwtick** global variable. This variable is used by various functions in the system (such as **Luos_GetSystick()**) and needs to be properly managed for the system to work.

## Use CMSIS RTOS interface to call kernel routines

Now we have to call kernel routines from our **main** function. We could directly use FreeRTOS functions, but ARM defined a standard API to interface with RTOS capabilities and we will use it. Follow the steps below to copy these files in your project: 

    cp -r ../tutorial_freertos/CMSIS_RTOS_V2/ ./lib/FreeRTOS/Source

We can call the OS API in our **main.c** file (notice that we deleted all the Luos API functions in **main**):

    #include "cmsis_os.h"

    int main(void)
    {
        HAL_Init();

        SystemClock_Config();

        osKernelInitialize();
        
        osKernelStart();
    }

**osKernelInitialize()** sets some FreeRTOS internal variables and **osKernelStart()** launches the scheduler by calling **vTaskStartScheduler()**.

One last thing to set the kernel, we have to tell to platformIO how to compile those new files. Open **platformio.ini** file at the root of your project and modify both following variables:

    build_flags =
        -I inc
        -include node_config.h
        -I lib//FreeRTOS//Source//include
        -I lib//FreeRTOS//Source//CMSIS_RTOS_V2
        -I lib//FreeRTOS//Source//portable//GCC//ARM_CM0
        -O1
    lib_deps = 
        Luos@^2.0.1
        LuosHAL
        Button
        FreeRTOS
        Led

We added the path to include files in **build_flags** and the source folder name in **lib_deps**.

Let's recall what we've done so far: 
- We have used a Luos button example as a base project.
- We imported FreeRTOS sources, configured them and started the kernel from the **main** function.

If you compile and load this code in the target, nothing will happen for a simple reason: we launched the scheduler but the kernel has nothing to schedule. Let's fix that by creating two Luos services in the dedicated FreeRTOS tasks (let's remind that we deleted all Luos related functions from the main).

## Create Luos tasks

We will create two services: a **button** service and a **led** service. The goal is to turn off / on the LED by pushing the user button on the Nucleo board. 

First, copy services' source files from the downloaded repository: 

    cp -r ../tutorial_freertos/Button/ ./lib/
    cp -r ../tutorial_freertos/Led/ ./lib/

Then we will call our services routines in FreeRTOS tasks, and we will manage that in a dedicated file: add the file called **freertos.c** in your project:

    cp ../tutorial_freertos/freertos.c ./src/

Open it in VSCode and you will see the following code: 

    #include "FreeRTOS.h"
    #include "task.h"
    #include "main.h"
    #include "cmsis_os.h"

    #include "luos.h"
    #include "button.h"
    #include "led.h"

    const osThreadAttr_t LuosTask_attributes = {
        .name       = "LuosTask",
        .stack_size = 128 * 4,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    const osThreadAttr_t ButtonTask_attributes = {
        .name       = "Button",
        .stack_size = 128 * 4,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    const osThreadAttr_t LedTask_attributes = {
        .name       = "Led",
        .stack_size = 128 * 4,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    void StartLuosTask(void *argument);
    void StartButtonTask(void *argument);
    void StartLedTask(void *argument);

    void MX_FREERTOS_Init(void);

    void MX_FREERTOS_Init(void)
    {
        Luos_Init();
        Button_Init();
        Led_Init();

        osThreadNew(StartLuosTask, NULL, &LuosTask_attributes);
        osThreadNew(StartButtonTask, NULL, &ButtonTask_attributes);
        osThreadNew(StartLedTask, NULL, &LedTask_attributes);
    }

    void StartLuosTask(void *argument)
    {
        while (1)
        {
            Luos_Loop();

            taskYIELD();
        }
    }

    void StartButtonTask(void *argument)
    {
        while (1)
        {
            Button_Loop();

            taskYIELD();
        }
    }

    void StartLedTask(void *argument)
    {
        while (1)
        {
            Led_Loop();

            taskYIELD();
        }
    }

First, we included files we need to call FreeRTOS and Luos APIs. Then we created three structures: **LuosTask_attributes**, **ButtonTask_attributes** and **LedTask_attributes**. Those structures are used by FreeRTOS to configure **heap** size and **priority** for each task.

Here we have three tasks:
- luos task: it's used to manage the messages passing between services
- button task: it's used to read the blue button's state present on the nucleo board and send a message to the led service
- led task: it's used to receive the messages from button service and turn ON / OFF the LED.

These tasks are initialized in **MX_FREERTOS_Init()** with the **osThreadNew()** routine. Notice that we called **Luos_Init()**, **Button_Init()** and **Led_Init()** just before creating threads.

We've created three threads for two services because the first one is dedicated to handle **luos** platform. The three threads have the *same priority* and that's important because the scheduler will place them in the *same queue*, running one after the other. 

One last thing, in each *task* routine, we call the service loop then we *yield* to the next threads. Indeed, FreeRTOS scheduler can be called in preemptive or cooperative mode. The timer 7 will call it each 1 millisecond, but this implies that you will switch from one thread to the other at this period. This can be way too slow for most applications and we can improve the reactivity of the system by calling **taskYield** after each service loop routine.

## Test your project

Now you can build your project in VSCode and load your project in the nucleo board. Once the board is flashed, push the button and you should see the green LED turn on: FreeRTOS is now running Luos and your services. 

You can use this project to develop your application: to create a new service, add the code in the **lib/** folder and instanciate it by creating a new task in **freertos.c**.

## Main advantages

From a FreeRTOS developer point of view, Luos brought you APIs to develop distributed applications with ease:
- Develop each application in a service,
- manage messages between services through luos API, and
- manage and monitor your network with our SaaS tools.

From a Luos developer point of view, FreeRTOS can help to manage services scheduling to comply with hard real-time constraints.