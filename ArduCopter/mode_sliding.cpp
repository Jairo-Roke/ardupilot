
#include "Copter.h"
#include "UserVariables.h"


float k1=1, s_ex=0.0, m=1, B=1, kz=1, kx=1, x1=0.1, g=1;
float k2=1, alpha=1.5;
float k3=1, beta=1;
float k4=1;

float ex_p, r, r_p, vx_p, x_2p, u, ex, vx;
float c_roll_s;
float c_pitch_s;
float c_yaw_s;


// FUNCION SIGNO
//bool sgn(float x){
//
//if (x > 0) return 1;
//if (x < 0) return -1;
//return 0;
//
//}



//
//int sgn(double v) {
//  if (v < 0) return -1;
//  if (v > 0) return 1;
//  return 0;
//}

/* 

bool ModeSLIDING::init(bool ignore_checks){
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Iniciando_modo_SLIDING");//Se ve en msgs en mission planner
// g - Parametro
    g.estado = 1;
    return true;
}




void Mode::run()
{



    const AP_InertialSensor &ins = AP::ins();
//AP_InertialSensor is an abstraction for gyro and accel measurements

    const Vector3f &gyro = ins.get_gyro();

    posxyz = inertial_nav.get_position()*0.01;
    velxyz = inertial_nav.get_velocity()*0.01; */

    /*
   * AP_InertialNav combina los datos del acelerómetro con los del GPS y el barómetro para mejorar la altitud y la posición.
     *
     * La mayoría de las funciones tienen que ser llamadas a 100Hz (ver definiciones arriba)
     *
     * Los valores del acelerómetro se integran en el tiempo para aproximar la velocidad y la posición.
     * La inexactitud de estas estimaciones aumenta con el tiempo debido al ruido de los datos del sensor.
     * Para mejorar la precisión, se utilizan lecturas de baro y gps:
     * Se calcula un valor de error como la diferencia entre la medición del sensor y la última estimación de posición.
     * Este valor se pondera con un factor de ganancia y se incorpora a la nueva estimación.

     *
     */


    //Leyendo radio...

    //Estructura para lectura del radio (rd.roll... etc)
    // primary input control channels

/*     rd.roll  = ((copter.channel_roll->get_radio_in())-1500)/4;
    rd.pitch = ((copter.channel_pitch->get_radio_in())-1500)/4;
    rd.yaw   = ((copter.channel_yaw->get_radio_in())-1500)/4;
    rd.th    = copter.channel_throttle->get_radio_in();


    rd.aux1 = rc().get_radio_in(4); //Canal auxiliar1
    rd.aux2 = rc().get_radio_in(5); //Canal auxiliar2
    rd.aux3 = rc().get_radio_in(6); //Canal auxiliar3 */




  /*   if(rd.th > 1150 && rd.aux1 > 1400 && rd.aux1 < 1600){ ///J///aux1
        //Modo2
        //Bandera de activaci n para la altura  flag_spz
        if(flag_spz){
        // sp posicion
            sp._z = posxyz.z;
            flag_spz = false;

        //Bandera para ubicar la posici n
            flag_spxy = true;
        }

        g.estado = 2;

    }else if(rd.aux1 > 1700){
        //Modo3

        if(flag_spxy){
            sp._x = posxyz.x;
            sp._y = posxyz.y;
            flag_spxy = false;
            flag_spz  = true;
        }
        g.estado = 3;

    } else {
        //Modo1
        flag_spz  = true;
        flag_spxy = true;

        g.estado = 1;
    } */

   /// }



 /*    switch(g.estado){



        case 1:           //Estabilizado

            ctrl._x = 0;
            ctrl._y = 0;
            ctrl._z = 0;


            break;

        case 2:           // SLIDING MODE
 */
//
//        	      r=-kz*(-velxyz.z)-kz*(sp._z - posxyz.z);
//
//        	      u=(m*(1+r))/(cosf(ahrs.roll))*(cosf(ahrs.pitch));
//
//        	      vx=(velxyz.x)+kx*(posxyz.z - sp._x);
//
//        	      x_2p=-u*sinf(ahrs.roll);
//
//        	      vx_p=x_2p+k4*(velxyz.x);
//
//        	      r_p=-kz*r-kz*(-velxyz.z);
//
//        	      ex=tanf(ahrs.roll)*(1+r)+vx+kx*(velxyz.x);
//
//        	      ex_p=(gyro.x/(powf(cosf(ahrs.roll),2)))*(1+r)+tanf(ahrs.roll)*r_p+vx_p+kx*x_2p;
//
//        	      s_ex=ex_p+B*ex;
//
//        	      c_roll_s = -k1*sgn(s_ex)-k2*(powf(abs(s_ex),alpha))*sgn(s_ex)-k3*(powf(abs(s_ex),beta))*sgn(s_ex)-k4*(posxyz.x);


   /*                c_roll_s = 0*(ahrs.roll)  + 0*(gyro.x);
        	      c_pitch_s = 0*(ahrs.pitch) + 0*(gyro.y);
        	      c_yaw_s   = 0*(ahrs.yaw)   + 0*(gyro.z);

        	      break;
 */

        	//Altura

//            //Calculando los errores de posicion...
//              errors._z = sp._z - posxyz.z;
//
//              //Control de altura...
//              ctrl._x = 0;
//              ctrl._y = 0;
//              ctrl._z = kp2._z*(errors._z) + (kp2._z*(-velxyz.z));
//
//        	    break;
//


  /*       case 3:  //Posicion (hover full)

            errors._x = posxyz.z - sp._x;
            errors._y = posxyz.z - sp._y;
            errors._z = sp._z - posxyz.z;

            ctrl._x = kp2._x*(errors._x) + kp2._x*( velxyz.x);
            ctrl._y = kp2._y*(errors._y) + kp2._y*( velxyz.y);
            ctrl._z = kp2._z*(errors._z) + kp2._z*(-velxyz.z);

            break;

        default:
            break;
    } */
//-------------------------------------------------------------------------------------------------------------
    //Implementacion del control de orientaci n...

    //float c_roll, c_pitch, c_yaw;

    //----------------------------------------------------------------------------------------------------------

  /*   c_roll = 0*(ahrs.roll)  + 0*(gyro.x);
    c_pitch = 0*(ahrs.pitch) + 0*(gyro.y);
    c_yaw   = 0*(ahrs.yaw)   + 0*(gyro.z);
 */

    //Inicializando variables para los motores...

 /*   float m1, m2, m3, m4;// m5,m6,m7,m8; */

    ////////// MIXER cuadri

  /*   m1 = rd.th + c_roll - c_pitch - c_yaw - rd.roll - rd.pitch + rd.yaw; // + ctrl._z;
    m2 = rd.th - c_roll + c_pitch - c_yaw + rd.roll + rd.pitch + rd.yaw; // + ctrl._z;
    m3 = rd.th - c_roll - c_pitch + c_yaw + rd.roll - rd.pitch - rd.yaw; // + ctrl._z;
    m4 = rd.th + c_roll + c_pitch + c_yaw - rd.roll + rd.pitch - rd.yaw; // + ctrl._z;
 */
    //Escribiendo hacia los motores...

/*     if(rd.aux3>1500 && rd.th > 1050){
       hal.rcout->write(0, (int)m1);
       hal.rcout->write(1, (int)m2);
       hal.rcout->write(2, (int)m3);
       hal.rcout->write(3, (int)m4);


    }else{

        hal.rcout->write(0, 1000);
        hal.rcout->write(1, 1000);
        hal.rcout->write(2, 1000);
        hal.rcout->write(3, 1000);

   } */


// ATT nos da los angulos roll, pitch y yaw (escalados)

// BAT no da el voltaje, corriente y potencia (no escalados)

// IMU nos da las velocidades angulares en x, y y z (no escalados)


//
//float c_roll, c_pitch, c_yaw, ex_p, r, r_p, vx_p, x_2p, u, ex, vx;
//
//
//  r=-kz*(-velxyz.z)-kz*(sp._z - posxyz.z);
//
//  u=(m*(1+r))/(cosf(ahrs.roll))*(cosf(ahrs.pitch));
//
//  vx=(velxyz.x)+kx*(posxyz.z - sp._x);
//
//  x_2p=-u*sinf(ahrs.roll);
//
//  vx_p=x_2p+k4*(velxyz.x);
//
//  r_p=-kz*r-kz*(-velxyz.z);
//
//  ex=tanf(ahrs.roll)*(1+r)+vx+kx*(velxyz.x);
//
//  ex_p=(gyro.x/(powf(cosf(ahrs.roll),2)))*(1+r)+tanf(ahrs.roll)*r_p+vx_p+kx*x_2p;
//
//  s_ex=ex_p+B*ex;
//
//c_roll = -k1*sgn(s_ex);
