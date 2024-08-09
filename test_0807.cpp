//ave_distance_error_sum 改成用90度最小值的誤差 save_robot_wall 離牆距離存檔 改回原本的
//1018 速度項改了
//1023 out_y[]初始化
///202011 time 170 to 100 to 170 202012 雷射方向改
///min_right 加40度 延牆0.35m
//////////////// include 區 /////////////////{ 
#include <assert.h>  
#include <math.h>
#include <stdio.h> 
#include <stdlib.h>
#include <iostream>
#include <webots/lidar.h> 
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include "time.h"
#include <algorithm> 
///////////////// include 區 //////////////////} 
 
//////////////////////////////  define區  ///////////////////////////////////////{

//讀檔路徑 in
//#define load_data_path    "..\\..\\..\\z.txt" 
//#define save_data_path01  "..\\..\\..\\Test01_SaveMotorSpeed_w.txt"  
//#define save_data_path02  "..\\..\\..\\Test01_Save_wall_distance_error_w.tt" 

#define load_data_path    "..\\..\\..\\1\\generated_file\\temp_check\\temp_w.txt"	  
//#define load_data_path_target    "..\\..\\..\\1\\controllers\\20230606\\real_positions.txt"
  
#define load_data_path_target    "..\\..\\..\\1\\controllers\\seek_PID\\real_positions.txt"    
#define save_data_path01  "..\\..\\..\\1\\generated_file\\speed-w-exp1.txt"
#define save_data_path03  "rms.txt"  
#define save_data_path02  "..\\..\\..\\1\\generated_file\\error-exp1.txt" 
#define save_data_path_pos  "..\\..\\..\\1\\generated_file\\pos-exp1-s1.txt" 


    
       
//雜訊輸入 
#define Gauss_noise_mean_value 0.0 
#define Gauss_noise_standard_deviation 0.05

//#define Tp 5   
	  
	  
///////////////////  演算法參數  /////////////////////{
#define _out_varl 6   /*  輸出變數 (左輪&右輪) */
#define _in_varl  3   /*  輸入變數    */
		 	   
#define _rule_number 10   /* 規則數 */ 

#define  _max_rule  30    /* 開記憶體空間 */

///////////////////  演算法參數  /////////////////////}

///////////////////  速度項目  /////////////////////{
#define left_wheel_speed 50.0 /* 左輪輪速 */
#define right_wheel_speed  50.0   /* 右輪輪速  */
#define robot_speed 0  /*  機器人的基礎速度  */
///////////////////  速度項目  /////////////////////}
 
///////////////////  距離項目  /////////////////////{


#define max_step 799    /* 最大步數  */  
#define _input_scale (1/sensor_range)  /* 給感測器讀值正規化的參數  */
#define getArRadius 0   /*  車體半徑  */
///////////////////  距離項目  /////////////////////{

/////////////////  生成隨機函數  /////////////////////{
#define randomize() srand((unsigned) time(NULL)) /*  以時間為基底產生隨機值的初始化，不要動這行  */
#define random(x) rand() %x   /*  隨機產生亂數，不要動這行  */
/////////////////  生成隨機函數  /////////////////////}


/////////////////  其他  /////////////////////{

#define TIME_STEP 32 /*  延遲時間 */
#define INF 1.0e14     /*  無限大  */

/////////////////  其他  /////////////////////}



//////////////////////////////  define區  ///////////////////////////////////////}


 
//////////////////////////////  變數宣告區  //////////////////////////////////////{

////////////////  感測器相關  //////////////////{

int counts=0;
double rms =0;
double in[_in_varl+1] ; //感測器正規化後的輸入
int margin_reset=0;
int final_counters = 0; 
float xx[max_step+1],yy[max_step+1]; //傳送座標資料時的暫存變數
////////////////  感測器相關  //////////////////}

////////////////  左右輪  //////////////////{
double error, turn ,intergral ,derivative,lastError,error_1 ,derivative_1 ,lastError_1,intergral_1;     
double out_y[_out_varl+1] ; //馬達輸出
float robot_vel[max_step+1] ;  // 機器人的左輪+右輪的速度
float robot_mov[max_step+1] ;  // 機器人的位移 ///202011
double deviation_whirl = 0.; //輪速差
double Tp;
////////////////  左右輪  //////////////////}

////////////////  步數相關  //////////////////{

int step,l=0 ;
double gg=0;

////////////////  步數相關  //////////////////}

////////////////  演算法相關  //////////////////{
double error1,h;
const int _mem_length = 2*_in_varl+ _out_varl ; // 一個rule的長度
const int real_mem = _rule_number*(2*_in_varl+ _out_varl) ;  // 整個解的個數
int _in_clu ; //rule_number的暫存變數	
double fuzzy_real[_mem_length*_max_rule+1] ; //  實際輸出規則的變數

//double error, turn ,intergral ,derivative,lastError;    
double K[_out_varl+1] ; //馬達輸出]
double right_speed=0 ,left_speed=0;
double angle_turn;
double  ave_motor_speed=0.0;  //平均輪速
double  ave_motor_displacement=0.0;  //平均位移 ///202011
double min_m[_in_varl+1],max_m[_in_varl+1];//最小中心點 & 最大中心點	

double Position_x[max_step+1],Position_y[max_step+1],Position_z[max_step+1]; //傳送座標資料時的暫存變數 
double robot_x,robot_y,sum_error,car_angle_z;
double Target_x[max_step+1],Target_y[max_step+1],Error_orientation_z[max_step+1];
double Trans_target_x,Trans_target_y,Trans_angle_z,Trans_degree,Trans_error_z; 
double error_x,error_y,error_z,total_distance;
double fff;
/////疊代紀錄的暫存空間///////{
int load_gen;  	
long double parent_rule_load[real_mem+1]={0.0};
/////疊代紀錄的暫存空間///////}

using namespace std;
////////////////  演算法相關  //////////////////}	

//其他
	
	
//////////////////////////////  變數宣告區  ///////////////////////////////////////}	
	
 
 	
//////////////////////////////  函式原型宣告區  ///////////////////////////////////////{	
	


///////////////////////  演算法相關  ////////////////////////{

void initialize_pop (); // 將解初始化

void Search_Target();

void speed_limit(void) ;   // 速度限制器

void robot_postion(int);  //抓取機器人位置

void execute_process();  //主執行序
double GaussDeviate(double , double);  //高斯重新取樣

///////////////////////  演算法相關  ////////////////////////}



///////////////////////  模糊相關  ////////////////////////{
void load_fuzzy_real();  //讀檔:讀取各個維度的fuzzy set 資料	

void In_mem_fir(double *,int) ; //計算激發量
void Fir_str(double *, int) ; //Fuzzy系統:讀取後件部與計算激發量
void fuzzy(int);    // Fuzzy系統:解模糊
void fuzzy_in(int) ;  // Fuzzy系統:把感測器的讀值做正規化

///////////////////////  模糊相關  ////////////////////////}


///////////////////////  存檔相關  ////////////////////////{
void save_robot_wheelspeed(int,int);  //左右輪輪速的存檔
void save_robot_wall_dis(int);  //離牆誤差的存檔
void save_robot_error(int);  //離牆距離的存檔  ///202011
void data_printf(int) ;  //印資料
void save_data(int);
///////////////////////  存檔相關  ////////////////////////}

////其他
      



//////////////////////////////  函式原型宣告區  ///////////////////////////////////////}

///////////////// 高斯函數 ////////////////////{

inline double phi(double x,double m,double v)
{ return(  exp( - (x-m)*(x-m)/(v*v) )) ; }

///////////////// 高斯函數 ////////////////////}

///////////////////////   class:C_Rule  /////////////////////////{

class C_Rule
{
friend class Population ;
public :

double  in_mu_1,in_mu_2 ;
double  con[_out_varl+1] ;
void    In_mem_fir(double *,int) ;
friend  void Fir_str(double *, int) ;

} ;

C_Rule  _Rule[_max_rule+1] ;

///////////////////////   class:C_Rule  /////////////////////////}

 
 
 //----------------------------------------  main區 ------------------------------------------------//{
 
int main(int argc, char **argv) 
{


 execute_process();
 wb_robot_step(TIME_STEP);  
 
 
wb_robot_cleanup();

  return 0;
}


//----------------------------------------  main區 ------------------------------------------------//}

 

/////////////////////////////  抓取機器人位置  //////////////////////////{
void robot_postion(int k)
{
 WbNodeRef robot_node = wb_supervisor_node_get_from_def("Tracer");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  
   const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
   //座標驗證用
 //   printf("MY_ROBOT is at position: %g %g %g\n", trans[0], trans[1], trans[2]);

//轉存:把trans[]矩陣存出去。

  double x, z;
  x = trans[0];
  z = trans[2];
 
   xx[k] = x;
   yy[k] = z;
  
  
   total_distance+=sqrt(pow((xx[k]-xx[k-1]),2)+pow((yy[k]-yy[k-1]),2));
 fflush(stdout);
}
/////////////////////////////  抓取機器人位置  //////////////////////////}

/////////////////////////////  清除矩陣資料  //////////////////////////{


///////////////////////////////////// 抓感測器 ///////////////////////////////{


/////////////////////////////////////  將解初始化   ///////////////////////////////////////{
// Step1:全部生成隨機的初始值-->Step2:指定特定的rule給特定值(內直角與外直角)
void initialize_pop()
{
    int j; 
////////////////////////////// Step 1: 生成全部的初始隨機解  ////////////////////////////////	

///////////// 矩陣內全部歸零 ////////////{		 
  for (j=1; j<=_max_rule*_mem_length; j++)
  {
      fuzzy_real[j] = 0. ;
  }
///////////// 矩陣內全部歸零 ////////////}
}

/////////////////////////////////////  將解初始化   ///////////////////////////////////////}
double slect_int(double k){
 int i;
 k=k*100;
 i=fmod(k,10);
 k=k/100 ;
 if (i<6) //0.00~0.05
 { k=k*100;
  k=floor(k);
  k=k/100;
 }
  
  else { //0.06~0.09
    k=k*100;
    k=ceil(k);
    k=k/100;
    }
   return k;
    
}
void target_calculate( )
{
  int i = 1;

  FILE *f_pro11;
  f_pro11 = fopen(load_data_path_target,"r");
  while(fscanf(f_pro11,"%lf \t %lf\n",&Target_x[i],&Target_y[i])==2){
    i++;
  }
  fclose(f_pro11);

}

///////////////// Fuzzy系統:把感測器的讀值做正規化 ///////////////////{
void fuzzy_in(int jj)
{ 
    double limit_error=0;
    cout<<"jj= "<<jj<<endl; 
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("Tracer");
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    WbFieldRef  rot_field  =  wb_supervisor_node_get_field(robot_node,"rotation");
    const double *trans=wb_supervisor_field_get_sf_vec3f(trans_field);
    const double *trans_2=wb_supervisor_field_get_sf_rotation(rot_field);
    


    Position_x[jj] = trans[2] ; 
    Position_y[jj] = trans[0] ;
    Position_z[jj] = trans_2[3] ;
    robot_x = Position_x[jj];
    robot_y = Position_y[jj];
    
    Target_x[0]=-16.681118;
    
    Target_y[0]= 2.246065;
 
    
    if(jj>1)
    {  rms +=  sqrt(pow(Target_x[jj-1]-Position_x[jj],2)+pow(Target_y[jj-1]-Position_y[jj],2));
       sum_error=sqrt(pow(Target_x[jj]-Position_x[jj],2)+pow(Target_y[jj]-Position_y[jj],2));
       fff+= sqrt(pow(Position_x[jj]-Position_x[jj-1],2)+pow(Position_y[jj]-Position_y[jj-1],2));
     }
     //if  ( sqrt(pow((16-Position_x[step]),2)+pow((exp(-0.05*14)*cos(14)-Position_y[step]),2))<=0.3 ) l=0;
    if(  Position_z[jj] > 1.5707 && Position_z[jj] < 3.14159 ) Position_z[jj]= -1.5707-( 3.14159-Position_z[jj]); //第四象限為90~180
    else Position_z[jj] = trans_2[3] +1.5707 ;            

    error_x = Target_x[jj] - Position_x[jj]  ;
    error_y = Target_y[jj] - Position_y[jj] ;                      
    error_z = atan(abs(error_x/error_y)) ;//計算機器人與目標點的角度差
    Trans_angle_z = error_z*180/3.14159 ;//將徑度轉角度。 
     
    //用X、Y分量差位置判別角度差   
    if ( error_x>0 && error_y >0)   Trans_angle_z =  Trans_angle_z;
    else if( error_x<0 && error_y >0)  Trans_angle_z = -Trans_angle_z;
    else if( error_x<0 && error_y <0)  Trans_angle_z = -180+Trans_angle_z;
    else if( error_x>0 && error_y <0)  Trans_angle_z = 180-Trans_angle_z; 	

   // 車頭方向 - 位置的角度差 = 車頭需要旋轉的角度 ,判別左旋轉或右旋轉較小的的角度
    if ((Trans_angle_z > 90) && (Position_z[jj]*180/3.14158) <-90) {
      Trans_degree = -360+abs(Trans_angle_z)+abs(Position_z[jj]*180/3.14159) ;
      //angle_turn = 1; 
    } //機器人角度位於90~180 ,目標-90~180
    else if ((Trans_angle_z <-90) && (Position_z[jj]*180/3.14158) >90) {
      Trans_degree = 360-abs(Trans_angle_z)-abs(Position_z[jj]*180/3.14159) ;
     // angle_turn = 1;
    }//機器人角度位於90~180 ,目標-90~180  
   
     else Trans_degree = (Trans_angle_z - Position_z[jj]*180/3.14159);
     Trans_error_z = Trans_degree*3.14159/180 ;            
    
    Error_orientation_z[jj] = Trans_error_z;
    in[1] = abs(Trans_error_z)/3.14159;
    in[2] = abs(Error_orientation_z[jj]-Error_orientation_z[jj-1])/3.14159; 
    in[3] = sum_error/0.5;
    
    cout<<"rms=========== "<<rms<<endl;
    cout<<"總角度差= "<<Trans_angle_z<<endl;
    cout<<"總角度差= "<<Trans_degree*3.14159/180 <<endl;
    //if (Trans_error_z<0.31831016)
     // Error_orientation_z[jj] = Trans_error_z;
   // else 
    ///  Error_orientation_z[jj] = 1/Trans_angle_z; 
    
   


}

///////////////// Fuzzy系統:把感測器的讀值做正規化 ///////////////////}

/////////////////////////////  PID  //////////////////////////}
void Search_Target(){     
    error =Trans_error_z;
    error_1 = in[3];
    derivative = in[2];
    intergral += error;
    intergral_1 += error_1;
    derivative_1 = error_1 - lastError_1;
    turn = K[1]*error + K[2]*intergral + K[3]*derivative ;
    Tp =  abs( K[4]*error_1+ K[5]*intergral_1+K[6]*derivative_1);
     printf("TP:Kp=%lf, Ki=%lf,Kd=%lf\t Turn:Kp=%lf, Kd=%lf Ki=%lf\n",K[4],K[5],K[6],K[1],K[2],K[3]);
     cout<<"derivative_1= "<<derivative_1<<endl;
    if (abs(error)<0.05) {   //5.7degree 內直線行走
   	  intergral=0;
      intergral_1=0;
      right_speed=Tp;
      left_speed=Tp;
      printf("直走\n");
  	} 
     else if ( (error>0.05 &&  angle_turn != 1) ){ //一般情況下正為右旋轉，但跨過180度時則相反  || (error<-0.05 &&  angle_turn == 1)
      //out_y[1] = Tp+abs(turn) ;          
     // out_y[2] = Tp-abs(turn) ; 
      printf("右轉\n");
      lastError=error;
      lastError_1=error_1;
      
    	 
    }
   else if ( (error<-0.05 &&  angle_turn != 1)  ){ //一般情況下負為左旋轉，但跨過180度時則相反 //|| (error>0.05 &&  angle_turn == 1)
    //  out_y[1] = Tp-abs(turn) ;
    //  out_y[2] = Tp+abs(turn) ; 
      printf("左轉\n");
      lastError=error; 
      lastError_1=error_1;
      
   }
    out_y[1] = Tp+turn;          
    out_y[2] = Tp-turn; 
   printf("intergral=%lf\n",intergral);
}
/////////////////////////////  PID  //////////////////////////}

////////////////////// Fuzzy系統:讀取後件部 //////////////////////////{
void   Fir_str(double *in, int _rule)     
{
//這裡的ber，等於main函式內的ber，又ber的值為popsize的值。
//這裡的 _rule，等於設定變數的_rule_number的值。
//這裡的in，代表輸入的變數的數量

int j,k ;

 for(k=1; k<=_rule; k++)  //_rule值=_rule_number=10
 {
   for (j=1; j<=_out_varl; j++)
   {
     // 將左右輪的後件部值做"轉存"
      _Rule[k].con[j] = fuzzy_real[k*_mem_length-_out_varl+j] ;  
  }
   _Rule[k].In_mem_fir(in,k) ; // 計算激發量
}
}
 
////////////////////// Fuzzy系統:讀取後件部 //////////////////////////}
void C_Rule::In_mem_fir(double *in,int _rule )  
{
//這裡的ber，等於main函式內的ber，又ber的值為popsize的值。
//這裡的 _rule，等於設定變數的_rule_number的值，而且這裡的_rule是一個動態變數，所以會隨著 Fir_str 的 k 值做改變。
  
int i ;
in_mu_1 =  1. ;  // "1." 代表1的float值
in_mu_2 =  1. ;
    for(i=1;i<= _in_varl;i++)
     { 
          
          in_mu_1 =  in_mu_1 * phi(in[i] , fuzzy_real[(_rule-1)*_mem_length+i*2-1] , fuzzy_real[(_rule-1)*_mem_length+i*2] ) ;
      }  
   //  printf("in_mu_1=%lf \tin_mu_2=%lf  \n",in_mu_1,in_mu_2);
    
    
}
////////////////////// Fuzzy系統:計算激發量 //////////////////////////}

//////////////////////// 解模糊 ///////////////////////////{
 // 權重式平均法
//這裡的 _rule，等於設定變數的_rule_number的值。
void fuzzy(int _rule ) 
{
int i , j;
double den[_out_varl+1] ,   //分母項
       num[_out_varl+1] ;   //分子項

 for (j=1; j<=_out_varl; j++)
 {
  den[j] = 0. ;  //初始化歸零
  num[j] = 0. ;  //初始化歸零
   
   for (i=1; i<=_rule; i++)
   {
   num[j] = num[j] + _Rule[i].in_mu_1 * _Rule[i].con[j];  //num為sigma的寫法
   den[j] = den[j] + _Rule[i].in_mu_1 ;   //den為sigma的寫法

   }

   if ( fabs(den[j]) < 1E-8 )
   K[j] = 0 ;     //如果den很小時
   else
   K[j] = num[j]/den[j] ;  //權重式平均法
 }
 
 
  // for (int kk=1; kk<=_out_varl; kk++) printf("den[%d]=%lf \t num=%lf\n",kk,den[kk],num[kk]);
}
//////////////////////// 解模糊 ///////////////////////////}

//////////////////////  速度限制器  //////////////////////////{
void speed_limit()
{
	// 使 Robot 維持在最高速及最低速
	if ( out_y[1] >= (  left_wheel_speed) )
		out_y[1] =  left_wheel_speed ;
	if ( out_y[2] >= (right_wheel_speed) )
		out_y[2] = right_wheel_speed ;
	if ( out_y[1] <= ( - left_wheel_speed) )
		out_y[1] =  - left_wheel_speed ;
	if ( out_y[2] <= ( - right_wheel_speed) )
		out_y[2] =  - right_wheel_speed ;	
}
//////////////////////  速度限制器  //////////////////////////}



//////////////////////  印資料   //////////////////////////{
void data_printf(int jj)
{ 
          
	printf(" ~~~~~~~~~~~~~~~~~~~~~~~~~~ \n" ) ;
  for (int i=1; i<=3;i++)
    cout<<"in["<<i<<"]=  "<<in[i]<<endl;
	printf("time step == %d\n", jj) ;
  printf("Tp=%lf\n",Tp);
	printf("左輪 == %f\n", out_y[1]) ;
	printf("右輪 == %f\n", out_y[2]) ;
	printf("車子==(%lf,%lf) \n",robot_x,robot_y) ;
  printf("目標==(%lf,%lf) \n",Target_x[jj],Target_y[jj]) ;
  printf("X座標差=%lf\n",Target_x[jj-1]-Position_x[jj]);
  printf("Y座標差=%lf\n",Target_y[jj-1]-Position_y[jj]);
  cout<<"角度差= "<<Trans_error_z<<endl;
  cout<<"目標-車子座標=\t"<< sqrt(pow((Target_x[jj]-Position_x[jj]),2)+pow((Target_y[jj]-Position_y[jj]),2))<<endl;
	fflush(stdout);	
}
//////////////////////////  印資料   //////////////////////////}


///////////////////////////////  主執行序  ///////////////////////////////{
void execute_process()
{

randomize() ;  // 以時間為基底產生隨機值的初始化



 /////// webot初始化 /////////{
  wb_robot_init();
 /////// webot初始化 /////////} 

  //抓取裝置
  WbDeviceTag left_wheel = wb_robot_get_device("left_motor");
  WbDeviceTag right_wheel = wb_robot_get_device("right_motor");
 
  
  


 ///////////////  馬達初始設定  //////////////{ 
  //讓馬達可以無限轉動的設定
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  
  //左右輪輪速的初始化
  wb_motor_set_velocity(left_wheel, left_speed);
  wb_motor_set_velocity(right_wheel, right_speed);
 ///////////////  馬達初始設定  //////////////}


_in_clu = 0 ; //rule_number的暫存變數 

	

initialize_pop(); //將解初始化

load_fuzzy_real() ;  //讀檔:讀取各個維度的fuzzy set 資料

    
		wb_motor_set_velocity(left_wheel , 0);
		wb_motor_set_velocity(right_wheel, 0);


    int step=0;
	
    _in_clu = _rule_number;  //rule_number的暫存變數
    out_y[1]=0;
    out_y[2]=0;
    double timer;
    target_calculate();
  while (wb_robot_step(100) != -1) 
     {    
          timer = wb_robot_get_time();
          step=step+1; 
          printf("親代執行區!!!!!!!\n");
          fuzzy_in(step) ;  //輸入是雷射數值，只是調整fuzzy的input比例(做正規化)			
          Fir_str(in , _rule_number) ; //讀取後件部計算每條rule機發量
          fuzzy(_rule_number); //解模糊產生out_y
          Search_Target();
          
          speed_limit() ; // 速度限制器
          data_printf(step) ; //印資料	
        
          FILE *f_pos;  
          if((f_pos=fopen(save_data_path_pos,"a"))==NULL) exit(1) ;           
          fprintf(f_pos,"%f\t %lf",Position_x[step],Position_y[step] ); 
          fprintf(f_pos , "\n");  
          fclose(f_pos); 

          ////////////////  馬達輸出  ///////////////////{
          wb_motor_set_velocity(left_wheel , out_y[1]);
          wb_motor_set_velocity(right_wheel, out_y[2]);
          wb_robot_step(200);
          ////////////////  馬達輸出  ///////////////////}
          ave_motor_speed += (out_y[1] + out_y[2])*0.5;
        
 
           //抓座標位置
         //  robot_postion(step); 
           
		   //save_robot_wall(step); //離牆距離的存檔

		  //停止條件  
		 if(step==max_step ||  robot_x>21 )  
		  {

			  save_robot_wheelspeed(step,timer);  //左右輪輪速的存檔     ///202011  
			  save_robot_wall_dis(step);    //離牆誤差的存檔   ///202011
			  save_robot_error(step); //位移的存檔 ///202011
			  wb_motor_set_velocity(left_wheel ,0.0);   
			  wb_motor_set_velocity(right_wheel,0.0); 
			  printf("total_angle_error=%lf\n",gg);
			  printf(" 停止，步數用盡 \n") ;        
        save_data(step);
        fflush(stdout);
        break;

       }
        cout<<"~~~~~~~~~~~~~~~~~~timer = "<<timer<<endl;
        cout<<"!!!!!!!!!!!!!!!!!!fff= "<<fff<<endl;
      //clear_matrix() ; // 清除陣列資料
      }
    
 } 


///////////////////////////////  主執行序  ///////////////////////////////}

  
  
//////////////////////////// 讀檔:讀取各個維度的fuzzy set 資料 /////////////////////////////{

void load_fuzzy_real()
{

 /////////////////  最小中心點 & 最大中心點 的初始化  ///////////////{
      for(int ssss=1; ssss<=_in_varl; ssss++)
      {      
        min_m[ssss]=1;
        max_m[ssss]=0; 
      } 
 /////////////////  最小中心點 & 最大中心點 的初始化  ///////////////} 
      
  
  //////////////////////////  規則讀檔  /////////////////////////{
  FILE *fnoise3;
   fnoise3 = fopen(load_data_path,"r"); 	
   
    for(int jj=1; jj<=real_mem; jj++)
    {     
        fscanf(fnoise3,"%Lf\t",&parent_rule_load[jj]);	        			
    }    
    fscanf(fnoise3, "\n") ; 
     
  fclose(fnoise3);
//////////////////////////  規則讀檔  /////////////////////////}
  
  
  
//////////////////////////  規則轉存  /////////////////////////{

    for(int jj=1; jj<=_rule_number; jj++)
    { 
        for(int jjj=1; jjj<=_in_varl; jjj++)
        {        
          fuzzy_real[(jj-1)*_mem_length+jjj*2-1] = parent_rule_load[(jj-1)*_mem_length+jjj*2-1] ; // 中心點
          fuzzy_real[(jj-1)*_mem_length+jjj*2] = parent_rule_load[(jj-1)*_mem_length+jjj*2] ; // 寬度
              
        }
        //////////////////////////////  計算左右輪的後件部  ////////////////////{
        fuzzy_real[jj*_mem_length-_out_varl+1] = parent_rule_load[jj*_mem_length-_out_varl+1];  //Kp
        fuzzy_real[jj*_mem_length-_out_varl+2] = parent_rule_load[jj*_mem_length-_out_varl+2];  //Kd
        fuzzy_real[jj*_mem_length-_out_varl+3] = parent_rule_load[jj*_mem_length-_out_varl+3];   //Ki
        fuzzy_real[jj*_mem_length-_out_varl+4] = parent_rule_load[jj*_mem_length-_out_varl+4];   //Ki
        fuzzy_real[jj*_mem_length-_out_varl+5] = parent_rule_load[jj*_mem_length-_out_varl+5];   //Ki
        fuzzy_real[jj*_mem_length-_out_varl+6] = parent_rule_load[jj*_mem_length-_out_varl+6];   //Ki
      //  if(    fuzzy_real[jj*_mem_length-_out_varl+4]<     fuzzy_real[jj*_mem_length-_out_varl+5])
       // {
       //       fuzzy_real[jj*_mem_length-_out_varl+4] = fuzzy_real[jj*_mem_length-_out_varl+5];
       //  }
     
         //////////////////////////////  計算左右輪的後件部  ////////////////////}
    }   
  
//////////////////////////  規則轉存  /////////////////////////}

  for(int jj=1; jj<=_rule_number; jj++)
  {
        for(int jjj=1; jjj<=_in_varl; jjj++)
        {   
          if(fuzzy_real[(jj-1)*_mem_length+jjj*2-1] < min_m[jjj])
          {   
            min_m[jjj]=fuzzy_real[(jj-1)*_mem_length+jjj*2-1];
          }
          if(fuzzy_real[(jj-1)*_mem_length+jjj*2-1] > max_m[jjj])
          {
            max_m[jjj]=fuzzy_real[(jj-1)*_mem_length+jjj*2-1];
          }
        }
  }
}
//////////////////////////// 讀檔:讀取各個維度的fuzzy set 資料 /////////////////////////////}




/////////////////////////////  左右輪輪速的存檔  //////////////////////////{
void save_robot_wheelspeed(int step,int timer)
{
  FILE *f_pro1;  
//if (step==max_step) ///202011
//{	
	if((f_pro1=fopen(save_data_path01,"a"))==NULL) exit(1) ;           
  fprintf(f_pro1," %lf\t %d\t %f\t %lf \t%lf",fff,step,(fff/(double)step),(ave_motor_speed/(double)step),(fff/timer)); 
	fprintf(f_pro1 , "\n");  
	fclose(f_pro1); 

}
void save_robot_error(int step)
{
  FILE *f_pro2;  
//if (step==max_step) ///202011
//{	
	if((f_pro2=fopen(save_data_path02,"a"))==NULL) exit(1) ;           
  fprintf(f_pro2,"%f",(rms/(double)step) ); 
	fprintf(f_pro2 , "\n");  
	fclose(f_pro2); 

}
/////////////////////////////  左右輪輪速的存檔  //////////////////////////}

/////////////////////////////  左右輪輪速的存檔  //////////////////////////{
void save_robot_wall_dis(int jj)
{
   FILE *pfout1;
     pfout1=fopen("../../controllers/error1.txt","a");
     if (pfout1 == NULL){
       printf("Fall to open file");
       exit(1);
     }
     fprintf(pfout1,"%lf \n",h/(double)jj);  
     fclose(pfout1);    
     cout<<"read file"<<endl;

}
void save_data(int jj){
   FILE *pfout0;
     pfout0=fopen("../../controllers/kk-s.txt","a");
     if (pfout0 == NULL){
       printf("Fall to open file");
       exit(1);
     }
     fprintf(pfout0,"%lf \t%lf \n",Position_x[jj],Position_y[jj]);  
     fclose(pfout0);    

}


/////////////////////////////  左右輪輪速的存檔  //////////////////////////}

/////////////////////////////  左右輪輪速的存檔  //////////////////////////}

///////////////////////////////  高斯重新取樣  ////////////////////////////{
double GaussDeviate(double mu, double sigma)
{
	double fac,r,v1,v2,x,new_var ;
	static int gaussSaved = 0; /* GaussDeviate generates numbers in pairs */
	static double gaussSave;    /* 2nd of pair is remembered here */
	
	if (gaussSaved) 
	{
		x = gaussSave; 
		gaussSaved = 0 ;
	}
	else 
	{
		do
		{
			v1 = 2.0*(double)rand()/RAND_MAX - 1.0 ;
			v2 = 2.0*(double)rand()/RAND_MAX - 1.0 ;
			r  = v1*v1 + v2*v2 ;
		}
		while (r>=1.0) ;
		
		fac = sqrt(-2.0*log(r)/r) ;
		gaussSaved = 1 ;
		gaussSave = v1*fac ;
		x = v2*fac ;
	}
	
	new_var = x * sigma + mu ;
	
	if (new_var > 1.0)
		new_var = 1.0 ;
	
	if (new_var < -1.0)
		new_var = -1.0 ;


	return new_var;
}
///////////////////////////////  高斯重新取樣  ////////////////////////////}











