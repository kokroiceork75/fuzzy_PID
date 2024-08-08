//2輸入，並且Tp使用輸入2(速度)，Turn使用輸入1(角度)，Tp:Kp:1~10， Kd:~1 Kp>Kd>Ki
///////////////// include 區 /////////////////{ 
#include <assert.h>  
#include <math.h>
#include <stdio.h>  
#include <stdlib.h>   
#include <webots/lidar.h> 
#include <webots/motor.h>  
#include <webots/robot.h>   
#include <webots/supervisor.h>   
#include "time.h"  
#include <algorithm> 
#include <iostream>
///////////////// include 區 //////////////////} 
  
/////////////////////////////  define區  /////////////////////////////////////{
 
///////////////////  演算法參數  /////////////////////{
#define _out_varl 6   /*  輸出變數 (Kp'Kd'Ki) */
#define _in_varl  3   /*  輸入變數    */
 
#define ngen  500/* 最大的世代數   */ 

#define popsize 40  /* 解的個數 */    ///202012
#define new_row 40  /* 新產生的解的個數  */	
		
#define _rule_number 10   /* 規則數 */ 

#define  _max_rule  20    /* 開記憶體空間用的 */
#define nobj 2           /*  目標個數，目前為2表示是2個目標  */

//#define Tp 5

#define   _pcross   0.9   /* 交配機率 */
#define   _pmutat   0.1  /* 突變機率 */

#define eta_c 10      /*  交配的變數  */
#define eta_m 10      /*  突變的變數  */
#define garma02 0.6     
#define garma03 0.75

///////////////////  演算法參數  /////////////////////}

/////////////////////  設定後件部、中心點與寬度的參數，用於資料更新  ///////////////////
 //限制後件部的範圍 
#define   _conq_max_Kp  50.0      /*  後件部最大值  */
#define   _conq_max_K  10       /*  後件部最大值  */
#define   _Turn_max_Kp  10      /*  後件部最大值  */
#define   _conq_min_Kp  0      /*  後件部最小值  */
#define   _conq_min_K  -10     /*  後件部最小值  */
#define   _Turn_min_K  -10     /*  後件部最小值  */
 // 限制寬度的範圍 
//#define   _range_max  0.4     /*  寬度最大值  */
//#define   _range_min  0.1     /*  寬度最小值  */ 
#define   _range_max  0.4     /*  寬度最大值  */
#define   _range_min  0.1     /*  寬度最小值  */ 
//限制中心點的範圍
#define   _center_max  1.   /*  中心點最大值  */
#define   _center_min  0.0    /*  中心點最小值   */ 
/////////////////////  設定後件部、中心點與寬度的參數，用於資料更新  ///////////////////}

///////////////////  速度項目  /////////////////////{
#define left_wheel_speed 50.0  /* 左輪輪速 */
#define right_wheel_speed  50.0   /* 右輪輪速  */

///////////////////  速度項目  /////////////////////}
 
///////////////////  距離項目  /////////////////////{
#define stop_max_angle 1.0  /* 停止的角度  */
#define stop_dis_narrow 0.2  /* ? */
//#define derivation_z  0.055        // angle_derivation 15degree
#define max_step 800      /* 最大步數  */  
#define max_tune_step 30   /*  重置點的步數  */
#define sensor_range 1 /* 感測的範圍:1m */
#define getArRadius 0   /*  車體半徑  */
#define max_step_to14 30


///////////////////  距離項目  /////////////////////{

/////////////////  生成隨機函數  /////////////////////{
#define randomize() srand((unsigned) time(NULL)) /*  以時間為基底產生隨機值的初始化，不要動這行  */
#define random(x) rand() %x   /*  隨機產生亂數，不要動這行  */
/////////////////  生成隨機函數  /////////////////////}


/////////////////  其他  /////////////////////{

#define TIME_STEP 32 /*  延遲時間 */
#define INF 1.0e14     /*  無限大  */
/////////////////  其他  /////////////////////}


#define load_data_path    "real_positions.txt"	   
#define save_data_path03  "rms.txt"
#define save_data_path04  "rms---1.txt"
//////////////////////////////  define區  ///////////////////////////////////////}

 
//////////////////////////////  變數宣告區  ///////////////////////////////////////{

////////////////  感測器相關  //////////////////{

double  sn1[max_step+1], sn2[max_step+1], sn3[max_step+1], sn4[max_step+1];
double rms ; 
double error_x,error_y,error_z;         
double in[_in_varl+1] ; //感測器正規化後的輸入
double margin_laser[271]={0};  //將感測器前挪後的安全距離的感測範圍 ///202012 181
int margin_reset=0;

int limit_counts=0;

////////////////  感測器相關  //////////////////}

////////////////  左右輪  //////////////////{
double error, turn ,intergral ,derivative,lastError,error_1 ,derivative_1 ,lastError_1,intergral_1;
double stop_vel;    
double out_y[_out_varl+1] ; //馬達輸出
float robot_vel[max_step+1] ;  // 機器人的左輪+右輪的速度
double deviation_whirl = 0.; //輪速差

////////////////  左右輪  //////////////////}

////////////////  PID Parameter  //////////////////{
double K[_out_varl+1] ; //馬達輸出]
double right_speed=0 ,left_speed=0;
////////////////  PID Parameter  //////////////////}


////////////////  步數相關  //////////////////

int step ;
double Tp,mini_step,total_error ;
int angle_turn=1000;
int remained_step, current_stop_step ;
//float total_sucess_step=0;
double  total_sucess_step[2*popsize+1]={0.0};  //分開紀錄每個解的成功步數
float total_fail_step=0;
int no_move_reset_counter=0;
double  fail_step_record[2*popsize+1]={0.0};  //分開紀錄每個解的失敗步數
////////////////  步數相關  //////////////////}

////////////////  演算法相關  //////////////////{

//規則相關
int c_gen,l=0;  //計數用的世代數,ngen是最大的世代數
int ber ; //代表第幾個解，此為一個替代的暫存變數
const int _mem_length = 2*_in_varl+ _out_varl ; // 一個rule的長度
const int real_mem = _rule_number*(2*_in_varl+ _out_varl) ;  // 整個解的個數
int _in_clu ; //rule_number的暫存變數	
double fuzzy_real[popsize+new_row+1][_mem_length*_max_rule+1] ; //  實際輸出規則的變數


//非主導排序相關
int *temp1=NULL ; //用在非主導排序的暫存變數
int front_size ; // 排除front1後，統計該回合產生多少個Front的轉存變數
int *temp_sort ; //用在非主導排序的暫存變數
int front_sort[2*popsize+1] ; //用在非主導排序的暫存變數
int sorted_crowding[2*popsize+1] ; //用在非主導排序的暫存變數

//多樣性相關
double temp_crowd[2*popsize+1][nobj+2] ; //排列解的多樣性的暫存變數
int mixed_pop_sorted[2*popsize+1] ; //排列子代解的多樣性的暫存變數

//其他
double min_m[popsize+1][_in_varl+1],max_m[popsize+1][_in_varl+1];//最小中心點 & 最大中心點		

double Position_x[max_step+1],Position_y[max_step+1],Position_z[max_step+1]; //傳送座標資料時的暫存變數 
double robot_x,robot_y,sum_error,car_angle_z;
double Target_x[max_step+1],Target_y[max_step+1],Error_orientation_z[max_step+1];
double Trans_target_x,Trans_target_y,Trans_angle_z,Trans_degree,Trans_error_z; 
int best_solution[nobj+1]={0}; //儲存最佳解的變數名稱

////////////////  演算法相關  //////////////////}	

////////////////  可解釋性的costfunction  //////////////////{
double costvalue_interpretable; //可解釋性的costvalue的暫存變數
double epsilon_sum[popsize+1];
double penality[popsize+1];  
double fuzzy_real_compare[_mem_length*_max_rule+1] ; //儲存"已排除重複規則"的暫存空間
double a11_[100],a33_[100];

////////////////  可解釋性的costfunction  //////////////////}

////////////////  疊代紀錄的暫存空間  //////////
////////{

int load_gen;  	
long double parent_rule_load[popsize+1][real_mem+1]={0.0};
long double parent_obj_load[popsize+1][nobj+1]={0.0};
int parent_rank_load[popsize+1]={0};

////////////////  疊代紀錄的暫存空間  //////////////////}

//////////////////////////////  變數宣告區  ///////////////////////////////////////}	
	
 	
//////////////////////////////  函式原型宣告區  ///////////////////////////////////////{	
	
////////////////// 傳資料&重置函數  ///////////////////{
void robot_postion(int);  //抓取機器人位置
void robot_postion_reset(int);  //重置機器人位置
void Search_Target();// 角度差計算 
////////////////// 傳資料&重置函數  ///////////////////}	

///////////////////////  演算法相關  ////////////////////////{

void evaluate_pop (int,int,int,int,int) ; // 計算pop:用來評分、計算cost value與限制population
void initialize_pop (); // 將解初始化
void update_newparent() ;    //更新親代
int tour_selection() ; //競爭法
void merge(); //混合

void update_pop(double , double , int) ;   //解的更新
void crossover(int , int , int) ; //交配
void mutation() ;    // 突變  

void check_and_combime_child_set(int);  //fuzzy set的合併
void child_pop_cluster(int,int,int); //子代群

void overlap_degree(int,int);  //重疊度的計算
void fuzzy_set_constraint(int,int);  //fuzzy set之間的距離限制

///////////////////////  演算法相關  ////////////////////////}

///////////////////////  排序相關  ////////////////////////{

void non_domination(int,int) ;  //非主導排序:排列解的好壞
void sort_on_obj(int,int) ;  //按照 cost value 將解排序
void sort_on_crowd(int , int) ; //排列解的多樣性
void sort_on_rank_crowd(int) ; //做排列子代解的多樣性

///////////////////////  排序相關  ////////////////////////}

///////////////////////  模糊相關  ////////////////////////{

void In_mem_fir(double *,int,int) ; //計算激發量
void Fir_str(double *, int,int) ; //Fuzzy系統:讀取後件部與計算激發量
void fuzzy(int,int);    // Fuzzy系統:解模糊
void fuzzy_in(int) ;  // Fuzzy系統:把感測器的讀值做正規化

///////////////////////  模糊相關  ////////////////////////}

///////////////////////  回傳相關  ////////////////////////{

void report_pop (FILE *fpt);   //回傳解
void report_best(int,int) ; //回傳最佳解
void report_pop_for_reading ( FILE *fpt) ;
///////////////////////  回傳相關  ////////////////////////}

///////////////////////  存檔相關  ////////////////////////{

void save_final_obj(int) ; //存最終fuzzy set解
void save_time(float total_time) ;  //儲存時間      
void save_final_pareto_front() ;  //存最後一代的 front 
void save_temp_obj(int) ; //存暫時fuzzy set解
///////////////////////  存檔相關  ////////////////////////}

////其他

void gauss_data_limit(int) ;  // 限制高斯函數資料範圍             se  
void clear_matrix(void) ; //清除矩陣資料
void speed_limit(void) ;   // 速度限制器的原型宣告
void data_printf(int,int,int,int) ;  //印資料
double randn(double, double); //產生隨機亂數

void print_gen_obj(int);  //印出親代的cost

void execute_process(int);  //主執行序

void load_gen_data();

int end_iteration_and_save_gen(int);

void load_save_parent_rule_and_relative_data();

double slect_int(double);

using namespace std ; 

 
//////////////////////////////  函式原型宣告區  ///////////////////////////////////////}

///////////////////////  class:Population  /////////////////////////{
class Population
{
friend class C_Rule ;
public :
 
double xreal[_max_rule*_mem_length];
double crowd_dist;
double objv[nobj+1] ;   //此為cost function的數量
double smoothly[2];
int sp[2*popsize+1] ;
int nsp ;
int sorted_front ;
double dist[nobj+2] ;
int sorted_crowd ;
int nnp ;
int np[2*popsize+1] ;
int front_rank ;
Population() ;
	
} ;

Population :: Population()
{
}

Population parent_pop[popsize+new_row+1] ; //親代空間
Population child_pop[popsize+new_row+1] ; // 子代空間
Population mixed_pop[2*popsize+1] ; // 混合與後續功能用來轉存的暫存變數
Population temp_pop[2*popsize+1] ; //非主導排序與後續功能用來轉存的暫存變數

///////////////////////  class:Population  /////////////////////////}

///////////////////////  class:Front  /////////////////////////{

class Front
{
public:
  int member[2*popsize+1] ;
  int number ;
} ;

Front _front[2*popsize+1] ;

///////////////////////   class:Front  /////////////////////////{

///////////////// 高斯函數 ////////////////////{

inline double phi(double x,double m,double v)
{ return(  exp( - (x-m)*(x-m)/(v*v) )) ; }

///////////////// 高斯函數 ////////////////////}

///////////////////////   class:C_Rule  /////////////////////////{

class C_Rule
{
friend class Population ;
public :

double  in_mu_1, in_mu_2 ;
double  con[_out_varl+1] ;
void    In_mem_fir(double *,int,int) ;
friend  void Fir_str(double *, int,int) ;
friend  void fuzzy(double *,int,int);
} ;

C_Rule  _Rule[_max_rule+1] ;

///////////////////////   class:C_Rule  /////////////////////////}

 
//////////////////////  設定後件部、中心點與寬度的下限，用於交配和突變  ///////////////////////{
float min_variable(int i)
{
float ge;
int j;
int n=2*_in_varl+_out_varl;
j=i%n;  

/*
EX:
若是輸入項2個，輸出項1個，10條規則。
則i的最大值是一個解(10條規則)之中的所有格子，在這裡是一個變動值。
j代表一個規則裡對應的格子內容的數量，如中心點、寬度or後件部。
注意，j是"餘數"，所以得到的值會比n小。
*/

      if( j==n || j==(n-1)||j==(n-2)|| j==(n-3)|| j==(n-4) || j==(n-5))  ///202102 j==9
    {
	  // 後件部的最小值設定
      //以此例來說，輸出項會是每個rule的最後一部分，所以如果整除n，就是後件部。
      if (j==n) ge=_conq_min_K;
      else if (j==(n-1)) ge=_conq_min_K;
      else if (j==(n-2)) ge=_conq_min_Kp;
      else if (j==(n-3)) ge=_Turn_min_K;
      else if (j==(n-4)) ge=_Turn_min_K;
      else if (j==(n-5)) ge=_Turn_min_K;
   }
   else if(j%2==1)
   {
	   // 中心點的最小值設定
	   ge=_center_min;
   }
   else 
   {
	   //寬度的最小值
	  ge=_range_min;
             
   }
   return ge;
}
//////////////////////  設定後件部、中心點與寬度的下限，用於交配和突變  ///////////////////////}

//////////////////////  設定後件部、中心點與寬度的上限，用於交配和突變  ///////////////////////{
float max_variable(int i)
{
   float ge;
   int j;
   int n=2*_in_varl+_out_varl;  
   j=i%n;
 
/*
EX:
若是輸入項2個，輸出項1個，10條規則。
則i的最大值是一個解(10條規則)之中的所有格子，在這裡是一個變動值。
j代表一個規則裡對應的格子內容的數量，如中心點、寬度or後件部。
注意，j是"餘數"，所以得到的值會比n小。
*/

    if( j==n || j==(n-1)||j==(n-2)|| j==(n-3)|| j==(n-4) || j==(n-5))  ///202102 j==9
   {
	  // 後件部的最大值設定
      //以此例來說，輸出項會是每個rule的最後一部分，所以如果整除n，就是後件部。
      if (j==n) ge=_conq_max_K;
      else if (j==(n-1)) ge=_conq_max_K;
      else if (j==(n-2)) ge=_conq_max_Kp;
      else if (j==(n-3)) ge=_Turn_max_Kp;
      else if (j==(n-4)) ge=_Turn_max_Kp;
      else if (j==(n-5)) ge=_Turn_max_Kp;
   }
   else if(j%2==1)
   {
	   // 中心點的最大值設定
	   ge=_center_max;
   }
   else
   {
     ge=_range_max;
    
 
   }
   return ge;
}
//////////////////////  設定後件部、中心點與寬度的上限，用於交配和突變  ///////////////////////}

 
 
 //----------------------------------------  main區 ------------------------------------------------//{
 
int main(int argc, char **argv) 
{

 load_gen_data();

 if(load_gen<=ngen)
 {
 
   if(load_gen==1)
   {
     execute_process(load_gen);  //主執行序
   
     load_gen = end_iteration_and_save_gen(load_gen);

     printf("final=%d\n",load_gen);
 
    wb_supervisor_world_reload();
    wb_robot_step(TIME_STEP);    
   }
 
   else
   {
 
     load_save_parent_rule_and_relative_data();    //親代參數
     execute_process(load_gen);   //主執行序
   
     load_gen = end_iteration_and_save_gen(load_gen);

     printf("final=%d\n",load_gen);
 
     wb_supervisor_world_reload();
     wb_robot_step(TIME_STEP);  
   }  
 } 
  wb_robot_cleanup();

  return 0;
}


//----------------------------------------  main區 ------------------------------------------------//}


//////////////////////////////  讀檔:親代相關資訊  //////////////////////////{
void load_save_parent_rule_and_relative_data()
{

FILE *fnoise1;  

  fnoise1 = fopen("parent_rank.txt","r"); 	
  for (int i=1; i<=popsize; i++)
  {
    fscanf(fnoise1,"%d\t",&parent_rank_load[i]);	        
    fscanf(fnoise1, "\n") ;
  }
  fclose(fnoise1);

  
//////////////////////////  目標轉存  /////////////////////////{
  for(int i=1; i<=popsize; i++ )
  {   
      parent_pop[i].front_rank=parent_rank_load[i];          
  }
//////////////////////////  目標轉存  /////////////////////////}



//////////////////////////  目標讀檔  /////////////////////////{
FILE *fnoise2;
   fnoise2 = fopen("parent_obj.txt","r"); 	
  
  for (int i=1; i<=popsize; i++) 
  {   
    for(int jj=1; jj<=nobj; jj++)
    {     
        fscanf(fnoise2,"%Lf\t",&parent_obj_load[i][jj]);	        			
    }    
    fscanf(fnoise2, "\n") ;
  }   
  fclose(fnoise2);
//////////////////////////  目標讀檔  /////////////////////////}


//////////////////////////  目標轉存  /////////////////////////{
  for(int i=1; i<=popsize; i++ )
  {
    for(int jj=1; jj<=nobj; jj++)
    {
        parent_pop[i].objv[jj]=parent_obj_load[i][jj];        
    }   
  }
//////////////////////////  目標轉存  /////////////////////////}


//////////////////////////  規則讀檔  /////////////////////////{
FILE *fnoise3;
   fnoise3 = fopen("parent_rule.txt","r"); 	
  
  for (int i=1; i<=popsize; i++) 
  {   
    for(int jj=1; jj<=real_mem; jj++)
    {     
        fscanf(fnoise3,"%Lf\t",&parent_rule_load[i][jj]);	        			
    }    
    fscanf(fnoise3, "\n") ;
  }   
  fclose(fnoise3);
//////////////////////////  規則讀檔  /////////////////////////}

//////////////////////////  規則轉存  /////////////////////////{
  for(int i=1; i<=popsize; i++ )
  {
    for(int jj=1; jj<=_rule_number; jj++)
    {
        for(int jjj=1; jjj<=_in_varl; jjj++) 
        {        
          parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2-1] = parent_rule_load[i][(jj-1)*_mem_length+jjj*2-1] ; // 中心點
          parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2] = parent_rule_load[i][(jj-1)*_mem_length+jjj*2] ; // 寬度
              
        }
         
        //////////////////////////////  計算左右輪的後件部  ////////////////////{
        parent_pop[i].xreal[jj*_mem_length-_out_varl+1] = parent_rule_load[i][jj*_mem_length-_out_varl+1]; // Kp
        parent_pop[i].xreal[jj*_mem_length-_out_varl+2] = parent_rule_load[i][jj*_mem_length-_out_varl+2]; // Kd
        parent_pop[i].xreal[jj*_mem_length-_out_varl+3] = parent_rule_load[i][jj*_mem_length-_out_varl+3]; // Ki
        parent_pop[i].xreal[jj*_mem_length-_out_varl+4] = parent_rule_load[i][jj*_mem_length-_out_varl+4]; // Kd
        parent_pop[i].xreal[jj*_mem_length-_out_varl+5] = parent_rule_load[i][jj*_mem_length-_out_varl+5]; // Ki
        parent_pop[i].xreal[jj*_mem_length-_out_varl+6] = parent_rule_load[i][jj*_mem_length-_out_varl+6]; // Ki
         //////////////////////////////  計算左右輪的後件部  ////////////////////}
    }   
  }
//////////////////////////  規則轉存  /////////////////////////}

fflush(stdout);
}

//////////////////////////////  讀檔:親代相關資訊  //////////////////////////}



//////////////////////////////  讀檔:疊代次數  //////////////////////////{
void load_gen_data()
{
 
FILE *fnoise1;  

  fnoise1 = fopen("gen_record.txt","r");
   
    fscanf(fnoise1,"%d\n",&load_gen);	
  
  fclose(fnoise1);


printf("load_in=%d\n",load_gen);


fflush(stdout);
}

//////////////////////////////  讀檔:疊代次數  //////////////////////////}

//////////////////////////////  終止的存檔:疊代次數  //////////////////////////{
int end_iteration_and_save_gen(int load_gen)
{

load_gen=load_gen+1;

FILE *fnoise1;  

  fnoise1 = fopen("gen_record.txt","w");
   	
  fprintf(fnoise1,"%d\n", load_gen) ;  
  fclose(fnoise1);


printf("end_iteration_and_save_gen=%d\n",load_gen);


fflush(stdout);


return load_gen;
}
//////////////////////////////  終止的存檔:疊代次數  //////////////////////////}


/////////////////////////////  抓取機器人位置  //////////////////////////{
void robot_postion(int k)
{
 WbNodeRef robot_node = wb_supervisor_node_get_from_def("Tracer");
 WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
 WbFieldRef  rot_field  =  wb_supervisor_node_get_field(robot_node,"rotation");
 const double *trans=wb_supervisor_field_get_sf_vec3f(trans_field);
 const double *trans_2=wb_supervisor_field_get_sf_rotation(rot_field);
   //座標驗證用
 //   printf("MY_ROBOT is at position: %g %g %g\n", trans[0], trans[1], trans[2]);

//轉存:把trans[]矩陣存出去。

  Position_x[k] = trans[2] ; 
  Position_y[k] = trans[0] ;
  Position_z[k] = trans_2[2] ;
  robot_x = Position_x[k];
  robot_y = Position_y[k];
  
   no_move_reset_counter =0;
   if(k>200){ ///202012
	   for(int ccc=1;ccc<=200;ccc++)
	   {
		   if( sqrt(pow(Position_x[k]-Position_x[k-ccc],2)+pow(Position_y[k]-Position_y[k-ccc],2)) < 2.0)///202012 && k>200
		   {
			   no_move_reset_counter++;
		   }
	   }
   }

 fflush(stdout);
}
/////////////////////////////  抓取機器人位置  //////////////////////////}


/////////////////////////////  重置機器人位置  //////////////////////////{
void robot_postion_reset(int state)
{
 WbNodeRef robot_node = wb_supervisor_node_get_from_def("Tracer");
 WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
 WbFieldRef  rot_field  =  wb_supervisor_node_get_field(robot_node,"rotation");


//指定條件的重置功能 
  printf(" 重置! \n") ;
      
  //因0.45m 故把位置都左移0.1m  ,4.62557 to 4.52557,4.3 to 4.2
  const double INITIAL[3] = { 2.24607, 0.0891353, -16.6811 };  
  //const double INITIAL[3] = { 1, 0.0972512, -18.82 };  //初
  
  const  double  INITIAL_ROT[4]  =  { -0.0117115 , -0.999795,0.0167324, 1.14159 };
  wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL);   
  wb_supervisor_field_set_sf_rotation(rot_field,  INITIAL_ROT);
  wb_supervisor_simulation_reset_physics();
  fflush(stdout);
}
/////////////////////////////  重置機器人位置  //////////////////////////}

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

//////////////////////  產生隨機亂數  ///////////////////////{
double randn(double nmin, double nmax)
{
	double rinv;
	rinv=nmax-nmin;
	return ( (double)(rand()%(RAND_MAX+1)) / (double)RAND_MAX )*rinv+nmin;
}

//////////////////////  產生隨機亂數  ///////////////////////}

 

/////////////////////////////  清除矩陣資料  //////////////////////////{


/////////////////////////////  清除矩陣資料  //////////////////////////}

/////////////////////////////////////  將解初始化   ///////////////////////////////////////{
// Step1:全部生成隨機的初始值-->Step2:指定特定的rule給特定值(內直角與外直角)
void initialize_pop()
{
    int i,j,jj,jjj; 
////////////////////////////// Step 1: 生成全部的初始隨機解  ////////////////////////////////	

///////////// 矩陣內全部歸零 ////////////{		
for (i=1; i<=popsize; i++)
{
  for (j=1; j<=_max_rule*_mem_length; j++)
      fuzzy_real[i][j] = 0. ;
}
///////////// 矩陣內全部歸零 ////////////}

////////////////////////////  生成中心點、寬度、左右輪的值  //////////////////////////
//這是三層的for迴圈，外層執行1次，內層執行全部。

for (i=1; i<=popsize; i++ )
{
 for (jj=1; jj<=_rule_number; jj++)
  {			
    for(jjj=1; jjj<=_in_varl; jjj++)
    {
      parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2-1] =( 1.0*(double)rand()/RAND_MAX)  ;  // 中心點
      /*
      EX:
      當i=1,jj=1時，jjj要從1~5執行一次。
      此時中心點的號碼為 parent_pop[1].xreal[0*12+(1~5)*2-1]=parent_pop[1].xreal[1、3、5、7、9]
      */
      //(double)rand()/RAND_MAX :產生[0,1]之間的解(包含1)。
      
      
      parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2] =  _range_min + (fabs(_range_max-_range_min))*(double)rand()/RAND_MAX ; // 寬度
      /*
      EX: 
      當i=1,jj=1時，jjj要從1~5執行一次。
      此時中心點的號碼為 parent_pop[1].xreal[0*12+(1~5)*2]=parent_pop[1].xreal[2、4、6、8、10]
      */  
    }				
   parent_pop[i].xreal[jj*_mem_length-_out_varl+1] =  10*((2.0*(double)rand()/RAND_MAX)-1); //Kp 的 -5~5的隨機值
   parent_pop[i].xreal[jj*_mem_length-_out_varl+2] =  10*((2.0*(double)rand()/RAND_MAX)-1); //Kd 的 -5~5的隨機值 
   parent_pop[i].xreal[jj*_mem_length-_out_varl+3] =  10*((2.0*(double)rand()/RAND_MAX)-1); //Ki 的 -5~5的隨機值 
   parent_pop[i].xreal[jj*_mem_length-_out_varl+4] =  (50.0*(double)rand()/RAND_MAX); //Kp 的 3~15.6的隨機值
   parent_pop[i].xreal[jj*_mem_length-_out_varl+5] =  10*((2.0*(double)rand()/RAND_MAX)-1); //Kd 的 -8~8的隨機值
   parent_pop[i].xreal[jj*_mem_length-_out_varl+6] =  10*((2.0*(double)rand()/RAND_MAX)-1);//Kd 的 -8~8的隨機值
  
  } 
}
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

  FILE *f_pro1;
  f_pro1 = fopen(load_data_path,"r");
  while(fscanf(f_pro1,"%lf \t %lf\n",&Target_x[i],&Target_y[i])==2){
    i++;
  }
  fclose(f_pro1);

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
 
    
    if(jj>=1)
      rms +=  sqrt(pow(Target_x[jj-1]-Position_x[jj],2)+pow(Target_y[jj-1]-Position_y[jj],2));
    
  
    FILE *f_pros;  
  
    if((f_pros=fopen(save_data_path03,"a"))==NULL) exit(1) ;           
    fprintf(f_pros," %lf",rms); 
    fprintf(f_pros , "\n");  
    fclose(f_pros); 
     sum_error=sqrt(pow(Target_x[jj]-Position_x[jj],2)+pow(Target_y[jj]-Position_y[jj],2));
     //if  ( sqrt(pow((16-Position_x[step]),2)+pow((exp(-0.05*14)*cos(14)-Position_y[step]),2))<=0.3 ) l=0;
    if(  Position_z[jj] > 1.5707 && Position_z[jj] < 3.14159 ) Position_z[jj]= -1.5707-( 3.14159-Position_z[jj]); //第四象限為90~180
    else Position_z[jj] = trans_2[3] + 1.5707 ;            

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

    cout<<"總角度差= "<<Trans_angle_z<<endl;
    cout<<"總角度差= "<<Trans_degree*3.14159/180 <<endl;
    //if (Trans_error_z<0.31831016)
     // Error_orientation_z[jj] = Trans_error_z;
   // else 
    //  Error_orientation_z[jj] = 1/Trans_angle_z; 
    
   


}
///////////////// Fuzzy系統:把感測器的讀值做正規化 ///////////////////}

////////////////////// Fuzzy系統:讀取後件部 //////////////////////////{

void   Fir_str(double *in, int _rule, int ber)     
{
//這裡的ber，等於main函式內的ber，又ber的值為popsize的值。
//這裡的 _rule，等於設定變數的_rule_number的值。
//這裡的in，代表輸入的變數的數量

int j,k ;

 for(k=1; k<=_rule; k++)  //_rule值=_rule_number=10
 {
   for (j=1; j<=_out_varl; j++)
   {
     _Rule[k].con[j] = fuzzy_real[ber][k*_mem_length-_out_varl+j] ; // 將左右輪的後件部值做"轉存"
       
  }
   _Rule[k].In_mem_fir(in,k,ber) ; // 計算激發量
}
}
////////////////////// Fuzzy系統:讀取後件部 //////////////////////////}

////////////////////// Fuzzy系統:計算激發量 //////////////////////////{
void C_Rule::In_mem_fir(double *in,int _rule , int ber)  
{
//這裡的ber，等於main函式內的ber，又ber的值為popsize的值。
//這裡的 _rule，等於設定變數的_rule_number的值，而且這裡的_rule是一個動態變數，所以會隨著 Fir_str 的 k 值做改變。
  
int i ;
in_mu_1 =  1. ;  // "1." 代表1的float值
in_mu_2 =  1. ;
    for(i=1;i<= _in_varl;i++)
    {
            in_mu_1 = in_mu_1 * phi(in[i] , fuzzy_real[ber][(_rule-1)*_mem_length+i*2-1] , fuzzy_real[ber][(_rule-1)*_mem_length+i*2] ) ;
            in_mu_2 = in_mu_2 * phi(in[i] , fuzzy_real[ber][(_rule-1)*_mem_length+i*2-1] , fuzzy_real[ber][(_rule-1)*_mem_length+i*2] ) ;
     }  
}

////////////////////// Fuzzy系統:計算激發量 //////////////////////////}

//////////////////////// 解模糊 ///////////////////////////
 // 權重式平均法
//這裡的 _rule，等於設定變數的_rule_number的值。
//這裡的ber，等於main函式內的ber，又ber的值為popsize的值。 
void fuzzy(int _rule , int ber) 
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
 
 

  //if(K[1]<max(K[2],K[3])) K[1]=max(K[2],K[2]);
 
 //  for (int kk=1; kk<=_out_varl; kk++) printf("den[%d]=%lf \t num=%lf\n",kk,den[kk],num[kk]);
 //if( (K[j]< K[j]+1)|| (K[j]< K[j+2]))   K[j] =max(K[j+1],K[j+2]);
}
//////////////////////// 解模糊 ///////////////////////////}

//////////////////////  速度限制器  //////////////////////////{
void speed_limit()
{
	// 使 Robot 維持在最高速及最低速
	if ( out_y[1] >= left_wheel_speed )
		out_y[1] = left_wheel_speed ;
	if ( out_y[2] >=  right_wheel_speed )
		out_y[2] = right_wheel_speed ;
	if ( out_y[1] <=  - left_wheel_speed )
		out_y[1] = - left_wheel_speed ;
	if ( out_y[2] <=  - right_wheel_speed )
		out_y[2] =  - right_wheel_speed ;	
}
//////////////////////  速度限制器  //////////////////////////}

//////////////////////  印資料   //////////////////////////{
void data_printf(int jj, int c_gen, int ber, int _in_clu)
{ 
          
	printf(" ~~~~~~~~~~~~~~~~~~~~~~~~~~ \n" ) ;
  for (int i=1; i<=3;i++)
    cout<<"in["<<i<<"]=  "<<in[i]<<endl;
	printf("iteration == %d\n", c_gen) ;
	printf("ber == %d\n", ber) ;
	printf("time step == %d\n", jj) ;
  printf("Tp=%lf\n",Tp);
	printf("左輪 == %f\n", out_y[1]) ;
	printf("右輪 == %f\n", out_y[2]) ;
	printf("車子==(%lf,%lf) \n",robot_x,robot_y) ;
  printf("目標==(%lf,%lf) \n",Target_x[jj],Target_y[jj]) ;
  printf("X座標差=%lf\n",Target_x[jj-1]-Position_x[jj]);
  printf("Y座標差=%lf\n",Target_y[jj-1]-Position_y[jj]);
  cout<<"角度差= "<<Trans_error_z<<endl;
  cout<<"目標-車子座標=\t"<< sqrt(pow((Target_x[jj-1]-Position_x[jj]),2)+pow((Target_y[jj-1]-Position_y[jj]),2))<<endl;
	cout<<"rms= "<<rms<<endl;
  fflush(stdout);	
}
//////////////////////  印資料   //////////////////////////}


////////////////////////////////////  計算costvalue的前置作業  ///////////////////////////////////////////{
void evaluate_pop (int c_gen, int ber, int _rule, int remained_step , int current_stop_step)
{
//c_gen:計數用的世代數,
//這裡的ber，等於main函式內的ber，又ber的值為popsize的值。
//這裡的 _rule，等於設定變數的_rule_number的值。
//remained_step:殘留步數
//current_stop_step:停止時的步數
	 
double _angle=0. , //angle  
       velocity_all=0. ,//速度項
       total_vel=0.;
double total_error_dis=0.;
double parallel_wall=0. ;
double a1=0. , a3=0., a4=0.;
	

////////////////////////// 計算機器人離牆距離的cost fun.部分  /////////////////////////////{
double angle_too_big =0;
//deviation_whirl=out_y[1]-out_y[2];

   for ( int w=1; w<=current_stop_step; w++)
  {
       velocity_all += robot_vel[w];
  }
  
    total_error_dis = rms;

  
  for ( int  w=current_stop_step; w<=current_stop_step; w++)
  {
        total_vel += robot_vel[w];
   
  }
 
   


  if (current_stop_step <= 1E-8) //若停止時的步數少於1*10的負8次方時..
  {
    a1 = 10000. ;
    a3 = 10000. ;		
    a4 = 10000. ;
  }
  else 
  {
    a1 = total_error_dis ; // 轉存角度項  
    a3 = velocity_all;  // 轉存速度項
    a4 = total_vel;
    
    total_sucess_step[ber] = total_sucess_step[ber] + current_stop_step; //記錄停止時的步數，當做成功步數 //加上[ber]

	//記錄失敗的步數，等於殘留步數
                   
	/*if( (current_stop_step + remained_step) == max_step_to14)
	{                 
	    total_fail_step = total_fail_step + 200*remained_step;  //20200410  把100改成200
            }
	else
	{
	    total_fail_step = total_fail_step + remained_step;
	}*/

	 total_fail_step = total_fail_step + remained_step;

     fail_step_record[ber] = fail_step_record[ber] + total_fail_step;	 //20200515 
	 if(a3 < 0.000001){a3=0.000001;} ///202011
  }

  if (c_gen > 1)  //情況不為第一代的時候
  {
    child_pop[ber].objv[1] = a1+ child_pop[ber].objv[1];
    child_pop[ber].objv[2] = a3 + child_pop[ber].objv[2];

	//child_pop[ber].smoothly[1] = a4 + child_pop[ber].smoothly[1];(有要考慮平滑才要)
	//printf("計算完child_pop[%d].smoothly[1]= %f\n",ber,child_pop[ber].smoothly[1]);
  }
  else  //情況恰為第一代的時候
  {
    parent_pop[ber].objv[1] = a1+parent_pop[ber].objv[1];

   parent_pop[ber].objv[2] = a3 + parent_pop[ber].objv[2];
	//parent_pop[ber].smoothly[1] = a4 + parent_pop[ber].smoothly[1];
	//printf("計算完parent_pop[%d].smoothly[1]= %f\n",ber,parent_pop[ber].smoothly[1]);
  }
printf("a1=== %f\n",a1);
printf("a3=== %f\n",a3);
//printf("a4=== %f\n",a4);


}
////////////////////////////////////  計算costvalue的前置作業  ///////////////////////////////////////////}


//////////////////////////  印出親代的cost  ////////////////////////{
void print_gen_obj(int gen)
{

   if(gen==1)  
   {
     printf("total_a1 = %f\n",parent_pop[ber].objv[1]);    
    // printf("total_a3 = %f\n",parent_pop[ber].objv[2]); 
   }    
   else if(gen>1) 
   {
      printf("total_a1 = %f\n",child_pop[ber].objv[1]);
   //   printf("total_a3 = %f\n",child_pop[ber].objv[2]);  
   }
     
     
 fflush(stdout);

}
//////////////////////////  印出親代的cost  ////////////////////////}

////////////////////////////  非主導排序  ////////////////////////////////////{
// 求每個個體(solution)所對應的Front number and Crowding distance
//功能:排序解的"好壞"

void non_domination(int sw , int gg)
{

//////////  定義變數  ///////////{
int i,j,k,w,m ;
double _fmin=0. , _fmax=0., next_obj=0. , pre_obj=0. ;
double _dist=0.;
int less, equal, more ;
int front_rank ; 
int pop_size=0 ;
//////////  定義變數  ///////////}

//////////////////  親代的轉存  ////////////////////{
  if (sw == 1)
  {
   pop_size = popsize ; //轉存解的數量
   
    for(i=1; i<=pop_size; i++)
    {
      for (j=1; j<=nobj; j++)
      {
       temp_pop[i].objv[j] = parent_pop[i].objv[j] ; //cost value的轉存
      }
    }
  }
//////////////////  親代的轉存  ////////////////////}
	
//////////////////  子代的轉存  ////////////////////{	
  else if (sw == 3)
  {
   pop_size = popsize + new_row ; //轉存解的數量
      
    for(i=1; i<=pop_size; i++)
    {
      for (j=1; j<=nobj; j++)
      {
       temp_pop[i].objv[j] = mixed_pop[i].objv[j] ; //cost value的轉存
      }
    }
  }
//////////////////  子代的轉存  ////////////////////}

//////////////  初始化、歸零  /////////////////{
  for (i=1; i<=pop_size; i++)
    for (j=1; j<=pop_size; j++)
      _front[i].member[j] = 0 ;

	
front_rank = 1 ;
w = 0 ;

  for (i=1; i<=pop_size; i++)
 {
 temp_pop[i].nnp = 0 ;
 temp_pop[i].nsp = 0 ;
 temp_pop[i].front_rank = 0 ;
 temp_pop[i].crowd_dist = 0 ;
 _front[i].number = 0 ;
 }
 //////////////  初始化、歸零  /////////////////}

///////////////////////////  第一個 front 的處理  //////////////////////////{
  for (i=1; i<=pop_size; i++)
  {
    for (j=1; j<=pop_size; j++)
    {
    less=0 ;
    equal=0 ; 
    more=0 ;
  
    ///////////////////  情況分類:大於、等於、小於  ///////////////{
    for (k=1; k<=nobj; k++) 
    {
      if (temp_pop[i].objv[k] < temp_pop[j].objv[k])
      {
      less ++ ;
      }
      else if (temp_pop[i].objv[k] == temp_pop[j].objv[k])
      {
      equal ++ ;
      }
      else
      {
      more ++ ;
      }
    }	
      ///////////////////  情況分類:大於、等於、小於  ///////////////} 
////////////////////  含有等號的處理  //////////////////////{    	
     if( less == 0 && equal != nobj) // p被支配的解
     {
     // 含有一個等號配上一個大於的情況
     temp_pop[i].nnp ++ ;  //計數器加1
     temp_pop[i].np[ temp_pop[i].nnp ] = j ;  // ??
     }
     else if (more == 0 && equal != nobj) // p支配的解
     {
     //含有一個等號配上一個小於的情況
     temp_pop[i].nsp ++ ;  //計數器加1
     temp_pop[i].sp[ temp_pop[i].nsp ] = j ;  //??
     }	
////////////////////  含有等號的處理  //////////////////////}      
    }

//////////////////////  記錄 front 1 //////////////////////{    
    if ( temp_pop[i].nnp == 0 ) 
    {
    //如果沒有"等號配上一個大於"的情況，那保證必定是小於的情況
    w ++ ; //計數器加1 
    temp_pop[i].front_rank = 1 ;  //  說明第i個解的排名是1 
    
    _front[1].member[w] = i ; // 記錄在第幾個 i 的時候，是第 w 個 front 1，因為 front 1 可以同時存在很多個 
    }
    
  _front[1].number = w ; //記下目前有幾個 front 1 的個數 
//////////////////////  記錄 front 1 //////////////////////}  
  }
///////////////////////////  第一個 front 的處理  //////////////////////////}
	
///////////////////// 找出第二個Front以後，含有等號的相對應的rank  /////////////////////{ 
	
  while ( _front[front_rank].member[1] != 0 )
  //這裡的0不能寫NULL，NULL是給指標用的.NULL在指標裡代表.0,無物,無值.
  //而在指標外就無意義了，不可NULL當作數字來用.
  //在 _front[front_rank].member[1] 的 front_rank值於此副程式的開頭定義成 1，
  //代表一定會找到一個 front 1 有"值"之後，才會進到這個迴圈。 
  //總和之意: 當 front 1 的個數不為零時，進入while迴圈          
  {
  temp1 = new int[2*popsize+1] ;
  w= 0 ;
  
    for (i=1; i<=_front[front_rank].number; i++)
    //front_rank值於此副程式的開頭定義成 1，而_front[front_rank].number表示記下目前有幾個 front 1 的個數
    {
      if ( temp_pop[ _front[front_rank].member[i] ].sp[1] != 0 )
      //這裡的0不能寫NULL，NULL是給指標用的.NULL在指標裡代表.0,無物,無值.
      //在指標外就無意義了，不可NULL當作數字來用.
      //front_rank值於此副程式的開頭定義成 1，所以 _front[front_rank].member[i] 是記錄在第幾個 i 的時候，是 front 1
      //sp[1]的意思是，含有一個等號配上一個小於的情況，所以計數器加 1 
      //所以總和之意:當第 i 個解為front 1 ，且在i的編號之前出現"含有一個等號配上一個小於的情況 "，其數值恰不為零，則進入迴圈。
      {
        for (j=1; j<=temp_pop[ _front[front_rank].member[i] ].nsp; j++)
        //"當第 i 個解為front 1 ，且在i的編號之前出現含有一個等號配上一個小於的情況 "的計數器數值拿來用
        {
          temp_pop[ temp_pop[ _front[front_rank].member[i] ].sp[j] ].nnp = temp_pop[ temp_pop[ _front[front_rank].member[i] ].sp[j] ].nnp - 1 ; 
          //??     
          if ( temp_pop[ temp_pop[ _front[front_rank].member[i] ].sp[j] ].nnp == 0 ) // 若該解沒有被任何解支配, 則分配Front rank
          {
            temp_pop[ temp_pop[ _front[front_rank].member[i] ].sp[j] ].front_rank = front_rank + 1 ;
            w ++ ; // 計數每個Front成員的個數
            temp1[w] = temp_pop[ _front[front_rank].member[i] ].sp[j] ; 
          }
        }
       }
     }
  
  front_rank = front_rank + 1 ; // front_rank
  
  for (j=1; j<= w; j++)
  _front[front_rank].member[j] = temp1[j] ;

  _front[front_rank].number = w ; 

////////  釋放 temp1 的記憶體  ///////{
  delete [] temp1 ;
  temp1 = 0 ;
////////  釋放 temp1 的記憶體  ///////}
  } 
///////////////////// 找出第二個Front以後，含有等號的相對應的rank  /////////////////////}

front_size = front_rank-1 ; // 排除front1後，統計該回合產生多少個Front的轉存變數


//////////// 將solutions按照Front rank排序前的轉存 ////////////////{

k=0 ;
  for (i=1; i<=pop_size; i++)
  front_sort[i] = 0 ; //歸零

  for (i=1; i<=front_size; i++)
  {
    for (j=1; j<=_front[i].number; j++)
    {
      k ++ ;
      front_sort[k] = _front[i].member[j] ;
    }
  }
  for (i=1; i<=pop_size; i++)
  temp_pop[ front_sort[i] ].sorted_front = i ;
  
//////////// 將solutions按照Front rank排序前的轉存 ////////////////}

/////////////////////////// crowding distance的計算  /////////////////////////////{

  for (front_rank =1 ; front_rank<=front_size; front_rank++)
  {
    for (m=1; m<=nobj; m++)
    {
      temp_sort = new int[2*popsize+1] ;
      sort_on_obj(front_rank , m);  //按照 cost value 排序
      
      _fmin = temp_pop[ temp_sort[1] ].objv[m] ;  //cost value 的最小值為 front 1 
      _fmax = temp_pop[ temp_sort[ _front[front_rank].number ] ].objv[m] ; //cost value 的最大值為最後一名的 front  	
      
      temp_pop[ temp_sort[1] ].dist[m] = INF ;  // front 1 的 crowding distance 定為無窮大
      temp_pop[ temp_sort[ _front[front_rank].number ] ].dist[m] = INF ; // 最後一名的 front 的 crowding distance 定為無窮大

      for (j=2; j<=(_front[ front_rank].number-1); j++)
      {
      next_obj = temp_pop[ temp_sort[j+1] ].objv[m] ;
      pre_obj = temp_pop[ temp_sort[j-1] ].objv[m] ;
      
        if ( fabs(_fmax-_fmin) <= 1E-8 )
        temp_pop[ temp_sort[j] ].dist[m] = INF ;
        else
        temp_pop[ temp_sort[j] ].dist[m] = (next_obj-pre_obj)/(_fmax-_fmin) ;  //??
      }   
      ////////  釋放 temp1 的記憶體  ///////{
      delete [] temp_sort ;
      temp_sort = 0 ;
      ////////  釋放 temp1 的記憶體  ///////}
    }  
/////////  累加一組解的所有cost value 的 crowding distance ////////{   
    for (i=1; i<= _front[front_rank].number; i++)
    {
    _dist = 0;
    
    for (m=1; m<=nobj; m++)
    _dist = _dist + temp_pop[ _front[front_rank].member[i] ].dist[m] ; //累加一組解的所有cost value 的 crowding distance
    
    temp_pop[ _front[front_rank].member[i] ].crowd_dist = _dist ; //轉存累加後的值
    }
//////////  累加一組解的所有cost value 的 crowding distance //////////}
  } 	
/////////////////////////// crowding distance的計算  /////////////////////////////}

//////////////////  親代的轉存  ////////////////////{
  if ( sw == 1)
  {
    for (i=1; i<=pop_size; i++)
    {
    parent_pop[i].nnp = temp_pop[i].nnp ; // 含有一個等號配上一個大於的計數器的轉存
    parent_pop[i].nsp = temp_pop[i].nsp ; // 含有一個等號配上一個小於的計數器的轉存
    parent_pop[i].sorted_front = temp_pop[i].sorted_front ; //將solutions按照Front rank排序前的轉存
    parent_pop[i].front_rank = temp_pop[i].front_rank ; //轉存 rank
    parent_pop[i].crowd_dist = temp_pop[i].crowd_dist ; //轉存 crowding distance ，用在sort_on_crowd的函式
    
      for (j=1; j<=nobj; j++)
      {
      parent_pop[i].dist[j] = temp_pop[i].dist[j]; //轉存累加的 crowding distance
      }
      
      for (j=1; j<= temp_pop[i].nsp; j++)
      {
      parent_pop[i].sp[j] = temp_pop[i].sp[j] ;  // 轉存含有一個等號配上一個小於的情況的數量解??
      }
    }
  }
//////////////////  親代的轉存  ////////////////////} 
 
//////////////////  子代的轉存  ////////////////////{ 
  else if (sw == 3)
  {
    for (i=1; i<=pop_size; i++)
    {
    mixed_pop[i].nnp = temp_pop[i].nnp ;// 含有一個等號配上一個大於的計數器的轉存
    mixed_pop[i].nsp = temp_pop[i].nsp ; // 含有一個等號配上一個小於的計數器的轉存
    mixed_pop[i].sorted_front = temp_pop[i].sorted_front ;//將solutions按照Front rank排序前的轉存
    mixed_pop[i].front_rank = temp_pop[i].front_rank ;//轉存 rank
    mixed_pop[i].crowd_dist = temp_pop[i].crowd_dist ;//轉存 crowding distance ，用在sort_on_crowd的函式
    
      for (j=1; j<=nobj; j++)
      {
       mixed_pop[i].dist[j] = temp_pop[i].dist[j] ;//轉存累加的 crowding distance
      }
    
      for (j=1; j<= temp_pop[i].nsp; j++)
      {
       mixed_pop[i].sp[j] = temp_pop[i].sp[j] ;// 轉存含有一個等號配上一個小於的情況的數量解??
      }
    }
  }
//////////////////  子代的轉存  ////////////////////}   
}
////////////////////////////  非主導排序  ////////////////////////////////////}

///////////////////////// 按照 cost value 排序  //////////////////////////{
void sort_on_obj(int front_rank , int m) 
{
//這裡的 m 是 nobj 。

int pass,i,j;
double hold;
double temp_obj[2*popsize+1][nobj+2] ;

  for (i=1 ; i<= _front[front_rank].number; i++) 
  {
   temp_obj[i][1] = _front[front_rank].member[i] ; // 轉存第幾個解
   temp_obj[i][2] = temp_pop[ _front[front_rank].member[i] ].objv[m] ; //轉存cost value		
  }
  
 
///////////////////////  排序，順序正確即保存  /////////////////////{      
  for (pass=1 ; pass<= _front[ front_rank].number-1; pass++)
  {
    for( i=1; i<= _front[ front_rank].number-1; i++)
    {
      if (temp_obj[i][2] > temp_obj[i+1][2] )
      {
        for (j=1; j<=2; j++)
        {
        //這裡使用變數的交換儲存技巧
        hold = temp_obj[i][j] ;
        temp_obj[i][j] = temp_obj[i+1][j] ;
        temp_obj[i+1][j] = hold;
        }
      }
    }
  }
///////////////////////  排序，順序正確即保存  /////////////////////}

///////////////////////  將排好的順序重新轉存  ///////////////////{
  for (i=1 ; i<= _front[front_rank].number; i++)
  {
  temp_sort[i] = temp_obj[i][1] ;
  }
///////////////////////  將排好的順序重新轉存  ///////////////////}
}
///////////////////////// 按照 cost value 排序  //////////////////////////}

//////////////////////////// //排列解的多樣性(親代)  ///////////////////////////////{
// 按照 crodwing distance 排序 (大至小)
void sort_on_crowd(int sw , int gg) 
{

int pass,i,j,k,s,h=0;
double hold;
int pop_size=0 ;

//////////////////  親代的轉存  ////////////////////{
  if (sw == 1)
  {
  pop_size = popsize ;

    for (i=1; i<=pop_size; i++)
    temp_pop[i].crowd_dist = parent_pop[i].crowd_dist ;
  }
//////////////////  親代的轉存  ////////////////////}	

//////////////////  子代的轉存  ////////////////////{	
  else if (sw == 3)
  {
  pop_size = popsize + new_row ;
  
  for (i=1; i<=pop_size; i++)
  temp_pop[i].crowd_dist = mixed_pop[i].crowd_dist ;
  }
//////////////////  子代的轉存  ////////////////////}

  for (i=1; i<=pop_size; i++)
  sorted_crowding[i] = 0 ;  //暫存變數的歸零
  
  for (k=1; k<=front_size; k++)
  {
    for (j=1; j<=popsize + new_row; j++)  //暫存變數的歸零
    {
    temp_crowd[j][1] = 0. ;
    temp_crowd[j][2] = 0. ;
    }
    
    for (j=1; j<=_front[k].number; j++)
    {
    temp_crowd[j][1] = _front[k].member[j] ;  //轉存解的個數
    temp_crowd[j][2] = temp_pop[ _front[k].member[j] ].crowd_dist ; //轉存crodwing distance
    }
    ///////////////////////  排序，順序正確即保存  /////////////////////{ 
    for (pass=1; pass<= _front[k].number-1; pass++)
    {
      for (i=1; i<= _front[k].number-1; i++)
      {
        if (temp_crowd[i][2] < temp_crowd[i+1][2])
        {
          for (j=1; j<=2; j++)
          {
           //這裡使用變數的交換儲存技巧
	hold = temp_crowd[i][j] ;
	temp_crowd[i][j] = temp_crowd[i+1][j] ;
	temp_crowd[i+1][j] = hold ;
          }
        }
      }
    }
    ///////////////////////  排序，順序正確即保存  /////////////////////}

    for (s=1; s<=_front[k].number; s++)
    {
     h ++ ;
     sorted_crowding[h] = temp_crowd[s][1] ; //將排序好的結果轉存  (尚未對應到popsize)
    }	
  }
  
   for (i=1; i<=pop_size; i++)  
   temp_pop[ sorted_crowding[i] ].sorted_crowd =  i ; //設定的數量以符合接下來的轉存，因為親代的數量與子代不同。
   
//////////////////  親代的轉存  ////////////////////{   
   if (sw == 1)
   {
   for (i=1; i<=pop_size; i++)
   parent_pop[i].sorted_crowd = temp_pop[i].sorted_crowd ;
   }
//////////////////  親代的轉存  ////////////////////}   

//////////////////  子代的轉存  ////////////////////{   
   else if (sw == 3)
   {
     for (i=1; i<=pop_size; i++)
     {
     mixed_pop[i].sorted_crowd = temp_pop[i].sorted_crowd ;
     }
    sort_on_rank_crowd(pop_size) ; //做排列子代解的多樣性 
   }
//////////////////  子代的轉存  ////////////////////}
}
//////////////////////////// //排列解的多樣性(親代)  ///////////////////////////////}

///////////////////////////   排列子代解的多樣性   ////////////////////////////{
void sort_on_rank_crowd(int pop_size) 
{

int pass,i,j;
double hold;
double temp_obj[2*popsize+1][nobj+2] ; //因為子代解至少會比親代多，所以創造2倍的空間

  for (i=1 ; i<= pop_size; i++) //注意這裡只存"前40個解"
  {
  temp_obj[i][1] = i ; //轉存解的個數
  temp_obj[i][2] = mixed_pop[i].sorted_crowd ;//轉存crodwing distance	
  }
 ///////////////////////  排序，順序正確即保存  /////////////////////{   
  for (pass=1 ; pass<= pop_size-1; pass++)
  {
    for( i=1; i<= pop_size-1; i++)
    {
      if (temp_obj[i][2] > temp_obj[i+1][2] )
      {
        for (j=1; j<=2; j++)
        {
         hold = temp_obj[i][j] ;
         temp_obj[i][j] = temp_obj[i+1][j] ;
         temp_obj[i+1][j] = hold;
        }
      }
    }
  }
 ///////////////////////  排序，順序正確即保存  /////////////////////}
 
  for (i=1 ; i<= pop_size; i++)
  {
  mixed_pop_sorted[i] = temp_obj[i][1] ; //排序完畢，轉存 
  }

}
///////////////////////////   排列子代解的多樣性   ////////////////////////////}

/////////////////////////////// 使用參數歸零  //////////////////////////////{
void update_newparent()
{
	int i,j,jj ;
	for(i=1;i<=popsize;i++)
		parent_pop[i] = mixed_pop[ mixed_pop_sorted[i] ] ;
	//child_pop initial
	for(i=1; i<=popsize+new_row; i++)
	{
		child_pop[i].front_rank = 0 ;
		child_pop[i].crowd_dist = 0 ;
				
		for (j=1; j<=_max_rule*_mem_length; j++)
			child_pop[i].xreal[j] = 0 ;

		for (j=1; j<=nobj; j++)
			child_pop[i].objv[j] = 0 ;
	}

	//temp_child initial
	for(i=1; i<=popsize+new_row; i++)
	{
		temp_pop[i].front_rank = 0 ;
		temp_pop[i].crowd_dist = 0 ;
		
		for (j=1; j<=_max_rule*_mem_length; j++)
			temp_pop[i].xreal[j] = 0 ;

		for (j=1; j<=nobj; j++)
			temp_pop[i].objv[j] = 0 ;
	}
	
	//mixed_pop initial
	for(i=1; i<=popsize+new_row; i++)
	{
		mixed_pop[i].front_rank = 0 ;
		mixed_pop[i].crowd_dist = 0 ;
		
		for (j=1; j<=_max_rule*_mem_length; j++)
			mixed_pop[i].xreal[j] = 0 ;

		for (j=1; j<=nobj; j++)
			mixed_pop[i].objv[j] = 0 ;
	}	
}
/////////////////////////////// 使用參數歸零  //////////////////////////////}

/////////////////////////////////   回傳最佳解    ////////////////////////////{
void report_best(int c_gen, int sw )
{
double sorted_temp[2*popsize+1][nobj+1] ;
int i,pass ;

 for (int j=1; j<=nobj; j++)
 {
	 int w = 1 ;
	 
  for (i=1; i<=popsize;i++)
  {  
	sorted_temp[i][j] = parent_pop[i].objv[j] ;  //轉存 cost value 
  } 
  
  for (pass=2; pass<=popsize-1; pass++)
  {
     if (sorted_temp[w][j] > sorted_temp[pass][j])   // 為何是比第一個costvalue??
     {
        best_solution[j] = pass ;
        
        w=pass;
     } 
  }
 
  
  if(w==1)
  {
    best_solution[j]=w; 
  }
 }
}
/////////////////////////////////   回傳最佳解    ////////////////////////////}


/////////////////////////////////    競爭法    ////////////////////////////////////////{
// Tournament Selection Candidates
// lower rank is selected
int tour_selection() 
{
int *tour_cand , *cand_rank;
double *cand_dis ;
int tour_member , j ;
int tour_size = 2 ;
		
tour_cand = new int[tour_size+1] ;
cand_rank = new int[popsize+1] ;
cand_dis = new double[popsize+1] ;

  for (j=1; j<=tour_size; j++)
  {
  tour_cand[j] = random(popsize) + 1 ;
  cand_rank[j] = parent_pop[ tour_cand[j] ].front_rank ;
  cand_dis[j] = parent_pop[ tour_cand[j] ].crowd_dist ;
  }
  
  if (cand_rank[1] == cand_rank[2])
  {
    if (cand_dis[1] >= cand_dis[2])
    tour_member = tour_cand[1] ;
    else
    tour_member = tour_cand[2] ;
  }
  else
  {
    if (cand_rank[1] < cand_rank[2])
    tour_member = tour_cand[1];
    else
    tour_member = tour_cand[2];
  }
delete [] tour_cand;
delete [] cand_rank;
delete [] cand_dis ;
tour_cand = 0 ;
cand_rank = 0 ;
cand_dis = 0 ;

return tour_member;
}
/////////////////////////////////    競爭法    ////////////////////////////////////////}

////////////////////////////////////  混合  ////////////////////////////////////{

//混合親代與子代解成為一個陣列
// Routine to merge two populations into one //
void merge()
{
int i, w;	
  for (i=1; i<=popsize; i++)
    mixed_pop[i] = parent_pop[i] ; 
	
  for (i=1, w=(popsize+1); i<=new_row; i++, w++)
    mixed_pop[w] = child_pop[i] ;

}
////////////////////////////////////  混合  ////////////////////////////////////}

//////////////////////////////////  限制高斯函數資料範圍   ///////////////////////////////{  
void gauss_data_limit(int kk)
{
int j,jj;
  for (jj=1; jj<=_rule_number; jj++)
  {
    for(j=1;j<=_in_varl;j++)
    {
	   if (child_pop[kk].xreal[(jj-1)*_mem_length+j*2-1] < _center_min)
      {
       child_pop[kk].xreal[(jj-1)*_mem_length+j*2-1] = _center_min ; // 限制中心點最小值
      }
 
      if (child_pop[kk].xreal[(jj-1)*_mem_length+j*2-1] > _center_max)
      {
         child_pop[kk].xreal[(jj-1)*_mem_length+j*2-1] = _center_max ; // 限制中心點最大值   
      }
	  
		
      if (child_pop[kk].xreal[(jj-1)*_mem_length+j*2] < _range_min)
      {
       child_pop[kk].xreal[(jj-1)*_mem_length+j*2] = _range_min ; // 限制寬度最小值
      }
 
      if (child_pop[kk].xreal[(jj-1)*_mem_length+j*2] > _range_max)
      {
         child_pop[kk].xreal[(jj-1)*_mem_length+j*2] = _range_max ; // 限制寬度最大值   
      }
    }		
  }
}
//////////////////////////////////  限制高斯函數資料範圍   ///////////////////////////////}


////////////////////////////   存最後一代的 front   //////////////////////////////{

void save_final_pareto_front()
{
int i,j ;
FILE *fpt_init;
	 
fpt_init = fopen("..\\..\\generated_file\\final_pareto_front00.txt","a") ;

  for (i=1; i<=popsize; i++) 
  {  
    for(j=1; j<=nobj;j++)  
	{
		fprintf(fpt_init,"%f\t", parent_pop[i].objv[j]) ;  //cost value
	}
	fprintf(fpt_init,"%d\r\n", parent_pop[i].front_rank); //排名
	
  } 
  
fclose(fpt_init) ;


FILE *fpt_init01;
fpt_init01 = fopen("..\\..\\generated_file\\final_pareto_front.txt","a") ; 

  for (i=1; i<=popsize; i++)
  {
    for(j=1; j<=nobj;j++) 
    {
	fprintf(fpt_init01,"%f\t", parent_pop[i].objv[j]) ;  //cost value		
    }
    fprintf(fpt_init01, "\r\n") ;
  }
fclose(fpt_init01) ;

}
////////////////////////////   存最後一代的 front   //////////////////////////////}


///////////////////////////// 印出解  //////////////////////////////////{
// Function to print the information of a population in a file //
void report_pop ( FILE *fpt)
{
int i, j , jj , jjj ;

for (i=1; i<=popsize; i++)
{
  for (j=1; j<=nobj; j++)
  {
   fprintf(fpt,"%.10f\t", parent_pop[i].objv[j]);
  }
   fprintf(fpt," , \t");
		
  for(jj=1; jj<=_rule_number; jj++)
  {
    for(jjj=1; jjj<=_in_varl; jjj++)
    {
    fprintf(fpt,"%.10f\t", parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2-1] ); // 中心點
    fprintf(fpt,"%.10f\t", parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2] ) ; // 寬度
    }
    
   fprintf(fpt,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+1] ) ;  // Kp
   fprintf(fpt,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+2] ) ;  // Kd  			
   fprintf(fpt,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+3] ) ;  // Ki
   fprintf(fpt,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+4] ) ;  // Kd  			
   fprintf(fpt,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+5] ) ;  // Ki   	
   fprintf(fpt,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+6] ) ;  // Ki
   
  }

fprintf(fpt," , \t");
fprintf(fpt,"%d\t", parent_pop[i].front_rank);
fprintf(fpt,"%.10f\n", parent_pop[i].crowd_dist);
}
return;
}
///////////////////////////// 印出解  //////////////////////////////////}

//////////////////////////////////// 存最終fuzzy set解  ////////////////////////////////////////{
//即為最佳解的fuzzy set

void save_final_obj(int j) 
{
	
if (j==1)
{	
int jj , jjj ; 
FILE *f_obj ;

  if( (f_obj=fopen("..\\..\\generated_file\\w.txt","a"))==NULL) exit(1) ;
  {
    for(jj=1; jj<=_rule_number; jj++)
    { 
      for(jjj=1; jjj<=_in_varl; jjj++)
      {
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2-1] ); // 中心點
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2] ) ; // 寬度
      }
      
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+1] ) ;  // Kp
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+2] ) ;  // Kd  
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+3] ) ;  // Ki
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+4] ) ;  // Kd  
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+5] ) ;  // Ki 
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+6] ) ;  // Ki
    }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
}

if (j==1)
{	
int jj ; 
FILE *f_obj ;

  if( (f_obj=fopen("..\\..\\generated_file\\cost_w.txt","a"))==NULL) exit(1) ;
  {
     for(jj=1; jj<=nobj;jj++)  
     {
         fprintf(f_obj,"%f\t", parent_pop[best_solution[j]].objv[jj]) ;  //cost value
     }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
}

if(j==2)
{
int jj , jjj ; 

FILE *f_obj ;

  if( (f_obj=fopen("..\\..\\generated_file\\s.txt","a"))==NULL) exit(1) ;
  {
    for(jj=1; jj<=_rule_number; jj++)
    { 
      for(jjj=1; jjj<=_in_varl; jjj++)
      {
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2-1] ); // 中心點
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2] ) ; // 寬度
      } 
  
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+1] ) ;  // Kp
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+2] ) ;  // Kd  
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+3] ) ;  // Ki
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+4] ) ;  // Kd  
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+5] ) ;  // Ki 
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+6] ) ;  // Ki
    }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
}

if (j==2)
{	
int jj ; 
FILE *f_obj ;

  if( (f_obj=fopen("..\\..\\generated_file\\cost_s.txt","a"))==NULL) exit(1) ;
  {
     for(jj=1; jj<=nobj;jj++)  
     {
         fprintf(f_obj,"%f\t", parent_pop[best_solution[j]].objv[jj]) ;  //cost value
     }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
}

if(j==3)
{
int jj , jjj ; 
FILE *f_obj ;
 
  if( (f_obj=fopen("..\\..\\generated_file\\i.txt","a"))==NULL) exit(1) ;
  {
    for(jj=1; jj<=_rule_number; jj++)
    { 
      for(jjj=1; jjj<=_in_varl; jjj++)
      {
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2-1] ); // 中心點
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2] ) ; // 寬度
      }
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+1] ) ;  // Kp
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+2] ) ;  // Kd  
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+3] ) ;  // Ki
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+4] ) ;  // Kd  
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+5] ) ;  // Ki
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+6] ) ;  // Ki
    }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
}

if (j==3)
{	
int jj ; 
FILE *f_obj ;

  if( (f_obj=fopen("..\\..\\generated_file\\cost_i.txt","a"))==NULL) exit(1) ;
  {
     for(jj=1; jj<=nobj;jj++)  
     {
         fprintf(f_obj,"%f\t", parent_pop[best_solution[j]].objv[jj]) ;  //cost value
     }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
}

}
//////////////////////////////////// 存最終fuzzy set解  ////////////////////////////////////////}



//////////////////////////////////// 存暫存fuzzy set解  ////////////////////////////////////////{
//即為最佳解的fuzzy set

void save_temp_obj(int j) 
{
	
if (j==1)
{	
int jj , jjj ; 
FILE *f_obj ;

  if( (f_obj=fopen("..\\..\\generated_file\\temp_check\\temp_w.txt","w"))==NULL) exit(1) ;
 // if( (f_obj=fopen("fuzzy_real_temp_obj01_best.txt","w"))==NULL) exit(1) ;
  {
    for(jj=1; jj<=_rule_number; jj++)
    { 
      for(jjj=1; jjj<=_in_varl; jjj++)
      {
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2-1] ); // 中心點
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2] ) ; // 寬度
      }
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+1] ) ;  // Kp
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+2] ) ;  // Kd  
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+3] ) ;  // Ki
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+4] ) ;  // Kd
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+5] ) ;  // Ki
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+6] ) ;  // Ki
    }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
}

if (j==1)
{	
int jj ; 
FILE *f_obj ;

  if( (f_obj=fopen("..\\..\\generated_file\\temp_check\\cost_temp_w.txt","w"))==NULL) exit(1) ;
  {
     for(jj=1; jj<=nobj;jj++)  
     {
         fprintf(f_obj,"%f\t", parent_pop[best_solution[j]].objv[jj]) ;  //cost value
     }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
}

if(j==2)
{
int jj , jjj ; 

FILE *f_obj ;

  if( (f_obj=fopen("..\\..\\generated_file\\temp_check\\temp_s.txt","w"))==NULL) exit(1) ;
  {
    for(jj=1; jj<=_rule_number; jj++)
    { 
      for(jjj=1; jjj<=_in_varl; jjj++)
      {
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2-1] ); // 中心點
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2] ) ; // 寬度
      }
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+1] ) ;  // Kp
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+2] ) ;  // Kd
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+3] ) ;  // Ki
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+4] ) ;  // Kd
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+5] ) ;  // Ki
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+6] ) ;  // Ki

    }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
}

if (j==2)
{	
int jj ; 
FILE *f_obj ;

  if( (f_obj=fopen("..\\..\\generated_file\\temp_check\\cost_temp_s.txt","w"))==NULL) exit(1) ;
  {
     for(jj=1; jj<=nobj;jj++)  
     {
         fprintf(f_obj,"%f\t", parent_pop[best_solution[j]].objv[jj]) ;  //cost value
     }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
}

if(j==3)
{
int jj , jjj ; 
FILE *f_obj ;
 
  if( (f_obj=fopen("..\\..\\generated_file\\temp_check\\temp_i.txt","w"))==NULL) exit(1) ;
  {
    for(jj=1; jj<=_rule_number; jj++)
    { 
      for(jjj=1; jjj<=_in_varl; jjj++)
      {
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2-1] ); // 中心點
      fprintf(f_obj,"%.10f\t", parent_pop[best_solution[j]].xreal[(jj-1)*_mem_length+jjj*2] ) ; // 寬度
      }
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+1]  ) ;    // Kp
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+2]  ) ;  //Kd
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+3]  ) ;  //Ki  
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+4] ) ;  // Kd
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+5] ) ;  // Ki
    fprintf(f_obj,"%.10f   \t",  parent_pop[best_solution[j]].xreal[jj*_mem_length-_out_varl+6] ) ;  // Ki
    }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
}

if (j==3)
{	
int jj ; 
FILE *f_obj ;

  if( (f_obj=fopen("..\\..\\generated_file\\temp_check\\cost_temp_i.txt","w"))==NULL) exit(1) ;
  {
     for(jj=1; jj<=nobj;jj++)  
     {
         fprintf(f_obj,"%f\t", parent_pop[best_solution[j]].objv[jj]) ;  //cost value
     }
  }
fprintf(f_obj,"\n") ;
fclose(f_obj) ;
} 
}
//////////////////////////////////// 存最終fuzzy set解  ////////////////////////////////////////}

////////////////////////////////////////////  子代群  ////////////////////////////////////////////{
void child_pop_cluster(int i ,int step,int c_gen)
{
    int ii,iii,jj,jjj;
	

      for(jj=1; jj<=_in_clu; jj++)
	  {
	      for(jjj=1; jjj<=_in_varl; jjj++)
		  {
		   /////////////////////////////////  轉存  ////////////////////////////////
        fuzzy_real[i][(jj-1)*_mem_length+jjj*2-1] = child_pop[i].xreal[(jj-1)*_mem_length+jjj*2-1] ; // 中心點
          //這裡執行"中心點轉存"的動作。
        fuzzy_real[i][(jj-1)*_mem_length+jjj*2] = child_pop[i].xreal[(jj-1)*_mem_length+jjj*2] ; // 寬度
          //這裡執行"寬度轉存"的動作。

		    /////////////////////////////////  轉存  ////////////////////////////////
		  
		  ///////////////  為了限制一組rule中的5個高斯函數的範圍  ////////////////{
          if(fuzzy_real[i][(jj-1)*_mem_length+jjj*2-1] < min_m[i][jjj])
          {
            min_m[i][jjj]=fuzzy_real[i][(jj-1)*_mem_length+jjj*2-1];  //如果比最大的最小值(min_m = 1)還小，就儲存最小中心點
          }
          
          if(fuzzy_real[i][(jj-1)*_mem_length+jjj*2-1] > max_m[i][jjj])
          {
            max_m[i][jjj]=fuzzy_real[i][(jj-1)*_mem_length+jjj*2-1];  //如果比最小的最大值(max_m = 1)還大，就儲存最大中心點，
          }
          ///////////////  為了限制一組rule中的5個高斯函數的範圍 ////////////////}    
		   
		  }
		  
      fuzzy_real[i][jj*_mem_length-_out_varl+1] =  child_pop[i].xreal[jj*_mem_length-_out_varl+1]  ;  // Kp
      fuzzy_real[i][jj*_mem_length-_out_varl+2] =  child_pop[i].xreal[jj*_mem_length-_out_varl+2]  ;  // Kd				
      fuzzy_real[i][jj*_mem_length-_out_varl+3] =  child_pop[i].xreal[jj*_mem_length-_out_varl+3]  ;  // Ki	
      fuzzy_real[i][jj*_mem_length-_out_varl+4] =  child_pop[i].xreal[jj*_mem_length-_out_varl+4]  ;  // Kd				
      fuzzy_real[i][jj*_mem_length-_out_varl+5] =  child_pop[i].xreal[jj*_mem_length-_out_varl+5]  ;  // Ki
      fuzzy_real[i][jj*_mem_length-_out_varl+6] =  child_pop[i].xreal[jj*_mem_length-_out_varl+6]  ;  // Ki
	   
    }

}
////////////////////////////////////////////  子代群  ////////////////////////////////////////////}


//////////////////////////////////////////  重疊度的計算  /////////////////////////////////////////{
void overlap_degree(int ber, int _rule)
{
	float k;
	double mr, epsilon=0.0, espsum=0.0, n;

	//////////////////////////  規則的處理  //////////////////////////{
	//轉存
	for(int i=1;i<=_mem_length*_rule_number;i++)
	{ 
		fuzzy_real_compare[i]=fuzzy_real[ber][i]; 
	}  
	//排除重複的規則
	for(int jjj=1; jjj<=_in_varl; jjj++)
	{
		for(int jj=1; jj<=_rule_number; jj++)
		{		
			for(int jjjj=jj+1; jjjj<=_rule_number; jjjj++)		
			{
				if((fuzzy_real_compare[(jj-1)*_mem_length+jjj*2-1]==fuzzy_real_compare[(jjjj-1)*_mem_length+jjj*2-1]) && (fuzzy_real_compare[(jj-1)*_mem_length+jjj*2]==fuzzy_real_compare[(jjjj-1)*_mem_length+jjj*2]) )
				{
					fuzzy_real_compare[(jjjj-1)*_mem_length+jjj*2-1]=0.0;
					fuzzy_real_compare[(jjjj-1)*_mem_length+jjj*2]=0.0;	
				}
			}
		}	
	}
	//////////////////////////  規則的處理  //////////////////////////}


	for(int jjj=1; jjj<=_in_varl; jjj++)
	{
		for(k=_center_min; k<=_center_max; k=k+0.1)  //從-1開始畫刻度，每個單位是0.1，每0.1就計算1次
		{     
			mr=0.0;
				
			for(int jj=1; jj<=_rule; jj++)
			{
			/////////////////////  間隔內的激發量和  ///////////////////{
				n=phi(k, fuzzy_real_compare[(jj-1)*_mem_length+jjj*2-1], fuzzy_real_compare[(jj-1)*_mem_length+jjj*2]);							 	 			
				mr=mr+n;
			/////////////////////  間隔內的激發量和  ///////////////////}
			}	

			if(mr<0.5)  //意義:下界至少有重疊
		    {
			    epsilon=fabs(mr-0.5);
		    }
		    else if(mr>1.5) //意義:上界要有重疊的極限
		    {
				epsilon=fabs(mr-1.5); 
		    }
		    else 
		    {
			   epsilon=0.000001; 	//20200710 			    
		    }  
			 		 
			espsum=espsum+epsilon;   
		 
		} // end k
	}
	epsilon_sum[ber]=espsum;
}
//////////////////////////////////////////  重疊度的計算  /////////////////////////////////////////} 


//////////////////////////////////  fuzzy set之間的距離限制  /////////////////////////////////{

void fuzzy_set_constraint(int ber, int _rule)
{
	
//目的:限制set之間中心點的距離不要過小

	double dis=0.,delta_1,g=0.,max=0.;
	double check_dis_variable;
	double savep;
	double length=0.0;
	int tn=0;

	//////////////////////////  規則的處理  //////////////////////////{
	//轉換
	for(int i=1;i<=_mem_length*_rule_number;i++)
	{ 
		fuzzy_real_compare[i]=fuzzy_real[ber][i];
	}  
	//排除重複的規則
	for(int jjj=1; jjj<=_in_varl; jjj++)
	{
		for(int jj=1; jj<=_rule_number; jj++)
		{		
			for(int jjjj=jj+1; jjjj<=_rule_number; jjjj++)		
			{
				if((fuzzy_real_compare[(jj-1)*_mem_length+jjj*2-1]==fuzzy_real_compare[(jjjj-1)*_mem_length+jjj*2-1]) && (fuzzy_real_compare[(jj-1)*_mem_length+jjj*2]==fuzzy_real_compare[(jjjj-1)*_mem_length+jjj*2]) )
				{
					fuzzy_real_compare[(jjjj-1)*_mem_length+jjj*2-1]=0.0;
					fuzzy_real_compare[(jjjj-1)*_mem_length+jjj*2]=0.0;	
				}
			}
		}	
	}
	//////////////////////////  規則的處理  //////////////////////////}

	for(int jj=1; jj<=_in_varl; jj++)
	{		
		for(int m=1;m<=_rule;m++)
		{
			savep=1.0;
		
			for(int n=1;n<=_rule;n++)
			{			
				//兩個被挑選出來的中心點距離差		
				check_dis_variable = fabs(fuzzy_real_compare[(m-1)*_mem_length+jj*2-1]-fuzzy_real_compare[(n-1)*_mem_length+jj*2-1]);

				if(check_dis_variable<savep && check_dis_variable!=0.0)
				{				
					savep=check_dis_variable;//取最小的						
					tn=n;//存下當時的號碼
				}		
			}
	
			length=savep;//轉存最小的距離差
			max=garma03*fuzzy_real_compare[(m-1)*_mem_length+jj*2];				 
			dis=length-max;

            if(dis<0)
			{
				delta_1=dis*dis;
			}
			else
			{
				delta_1=0.0;
			}
            
			g=delta_1+g;	  
		}	    
	}
	penality[ber]=g;
}
//////////////////////////////////  fuzzy set之間的距離限制  /////////////////////////////////}



///////////////////////////////  主執行序  ///////////////////////////////{
void execute_process(int c_gen)
{
 int i=0,
     j,
     kk,  
     sw,   //sw : 作用是把第一代和子代分開，用在排序。
     jj,
     jjj ;

int iterate_counter;
int front_one_counter=0; //計算 front one 的個數的計數器
	 
FILE *fpt4,*fpt6;  //提供存檔的指標變數

randomize() ;  // 以時間為基底產生隨機值的初始


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
  //double left_speed = 0.0, right_speed = 0.0;
  wb_motor_set_velocity(left_wheel, left_speed);
  wb_motor_set_velocity(right_wheel, right_speed);
 ///////////////  馬達初始設定  //////////////}


_in_clu = 0 ; //rule_number的暫存變數



if(c_gen==1)
{	

initialize_pop(); //將解初始化

 /////////////////  最小中心點 & 最大中心點 的初始化  ///////////////{
    
 for(int ss=1; ss<=popsize; ss++ )
 {
  for(int ssss=1; ssss<=_in_varl; ssss++)
  {
        min_m[ss][ssss]=1;  //為了找最小值，所以初始的最小值令為1，即為"最大"的最小值。
        max_m[ss][ssss]=0;  //為了找最大值，所以初始的最大值令為0，即為"最小"的最大值。
     
       //驗證用
       //printf("min_m[%d][%d]:%f\n",ss,ssss,min_m[ss][ssss]); 
  }
 }
 /////////////////  最小中心點 & 最大中心點 的初始化  ///////////////}  

//////////////////////////  轉存&計算後件部  /////////////////////////{
  for(i=1; i<=popsize; i++ )
  {
    for(jj=1; jj<=_rule_number; jj++)
    {
        for(jjj=1; jjj<=_in_varl; jjj++)
        {        
          fuzzy_real[i][(jj-1)*_mem_length+jjj*2-1] = parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2-1] ; // 中心點
          /*
          這裡執行"中心點轉存"的動作。
          EX:當i=1，jj=1時，
          fuzzy_real[1][0*12+(1~5)*2-1]=fuzzy_real[1][1、3、5、7、9]=parent_pop[1].xreal[1、3、5、7、9]
          */
          fuzzy_real[i][(jj-1)*_mem_length+jjj*2] = parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2] ; // 寬度
          /*
          這裡執行"寬度轉存"的動作。
          EX:當i=1，jj=1時，
          fuzzy_real[1][0*12+(1~5)*2]=fuzzy_real[1][2、4、6、8、10]=parent_pop[1].xreal[2、4、6、8、10]
          */
          
          ///////////////  為了限制一組rule中的5個高斯函數的範圍  ////////////////{
          if(fuzzy_real[i][(jj-1)*_mem_length+jjj*2-1] < min_m[i][jjj])
          {
            min_m[i][jjj]=fuzzy_real[i][(jj-1)*_mem_length+jjj*2-1];  //如果比最大的最小值(min_m = 1)還小，就儲存最小中心點
          }
          
          if(fuzzy_real[i][(jj-1)*_mem_length+jjj*2-1] > max_m[i][jjj])
          {
            max_m[i][jjj]=fuzzy_real[i][(jj-1)*_mem_length+jjj*2-1];  //如果比最小的最大值(max_m = 1)還大，就儲存最大中心點，
          }
          ///////////////  為了限制一組rule中的5個高斯函數的範圍 ////////////////}      
        }
        //////////////////////////////  計算左右輪的後件部  ////////////////////{
        fuzzy_real[i][jj*_mem_length-_out_varl+1] =  parent_pop[i].xreal[jj*_mem_length-_out_varl+1]  ; // Kp
        fuzzy_real[i][jj*_mem_length-_out_varl+2] =  parent_pop[i].xreal[jj*_mem_length-_out_varl+2] ;  // Kd
        fuzzy_real[i][jj*_mem_length-_out_varl+3] =  parent_pop[i].xreal[jj*_mem_length-_out_varl+3] ;  // Ki
        fuzzy_real[i][jj*_mem_length-_out_varl+4] =  parent_pop[i].xreal[jj*_mem_length-_out_varl+4] ;  // Kd
        fuzzy_real[i][jj*_mem_length-_out_varl+5] =  parent_pop[i].xreal[jj*_mem_length-_out_varl+5] ;  // Ki
        fuzzy_real[i][jj*_mem_length-_out_varl+6] =  parent_pop[i].xreal[jj*_mem_length-_out_varl+6] ;  // Ki
        //////////////////////////////  計算左右輪的後件部  ////////////////////}
      //  printf("中心點:%lf\n",fuzzy_real[i][(jj-1)*_mem_length+jjj*2-1]);
      //  printf("寬:%lf\n", fuzzy_real[i][(jj-1)*_mem_length+jjj*2]);
      //  printf("Kp:%lf\n",fuzzy_real[i][(jj-1)*_mem_length+jjj*2+ 1]);
      //  printf("Kd:%lf\n", fuzzy_real[i][jj*_mem_length-_out_varl+2] );
      //  printf("Ki:%lf\n", fuzzy_real[i][jj*_mem_length-_out_varl+3]);

    }   
  }
//////////////////////////  轉存&計算後件部  /////////////////////////}

 
/////////////////////////////  親代:重置功能與 cost fun. 值的計算  ///////////////////////////////{

////////////////////////  親代主執行區  //////////////////////

  ber=0;
  
  while (ber<popsize) 
  {
    double timer ;
    mini_step=0.; 
    rms=0; 
    wb_motor_set_velocity(left_wheel , 0);
    wb_motor_set_velocity(right_wheel, 0);
    ber=ber+1;

    parent_pop[ber].objv[1]=0; // 將親代的第一個 cost value歸零。如果沒有5位置訓練要先歸零(有的話則不用)
    parent_pop[ber].objv[2]=0; // 將親代的第二個 cost value歸零。如果沒有5位置訓練要先歸零(有的話則不用)
	//parent_pop[ber].smoothly[1]=0; // 平滑用。如果沒有5位置訓練要先歸零(有的話則不用=0)
    //total_sucess_step=0.; //成功的步數歸零  
    fail_step_record[ber]=0;  //如果沒有5位置訓練要先歸零(有的話則不用)
    total_sucess_step[ber]=0; //如果沒有5位置訓練要先歸零(有的話則不用)
    total_fail_step=0;  //失敗的步數歸零  
    //total_error =0; 

    target_calculate();
    printf("ber = %d\n",ber);

    int step=0;
    int nod=0;
    int final_counter=0;
    
    limit_counts=0;
    error=0, turn=0 ,intergral=0 ,derivative=0,lastError=0,error_1=0 ,derivative_1=0 ,lastError_1=0;   
    robot_postion(step);///202011
    remained_step = 0 ; //殘留步數
    current_stop_step = max_step ;   //max_tune_step:重置點的步數 
    
    //停止的步數=重置點的步數
    _in_clu = _rule_number;  //rule_number的暫存變數
    int far_reset_counter = 0; //距離過遠而重置的計數器
    out_y[1]=0;///202011 第一步的第五個輸入要為0
    out_y[2]=0;
     l=0;
    timer =0;
     while (wb_robot_step(100) != -1 ) 
     { 
        timer = wb_robot_get_time(); 
        step=step+1;  

       //意即，跑到重置點步數前不會跳出迴圈，但同時會累計重置計數器。	
        printf("\n親代執行區!!!!!!!\n");
        
        fuzzy_in(step) ;  //輸入是雷射數值，只是調整fuzzy的input比例(做正規化)	
        Fir_str(in , _rule_number , ber) ; //讀取後件部計算每條rule機發量
        fuzzy(_rule_number , ber); //解模糊產生out_y

        Search_Target(); 
        speed_limit() ; // 速度限制器
        robot_vel[step] =(out_y[1]+out_y[2])/2;
        
        printf("robot_vel[step]=%lf",robot_vel[step]);

        if(robot_vel[step]<0)
	      {
      	  robot_vel[step]=0;
	      }
        data_printf(step, c_gen, ber, _in_clu) ; //印資料	

        ////////////////  馬達輸出  ///////////////////{
        wb_motor_set_velocity(left_wheel , out_y[1]);
        wb_motor_set_velocity(right_wheel, out_y[2]);
        wb_robot_step(200);
        ////////////////  馬達輸出  ///////////////////}

        
          if(( abs(Trans_error_z)> 3.14159 || limit_counts>=5) && step!=5 )  far_reset_counter=11;
          
          else if(abs(Trans_error_z)> stop_max_angle)
          {
             far_reset_counter++;
          } 
          else {
            far_reset_counter--;

            if (far_reset_counter<0) 
             far_reset_counter=0;

          }
        
    
        
           //抓座標位置
          robot_postion(step); 
          printf("far_reset_counter=%d\n",far_reset_counter);
          
          current_stop_step =step;
            if ( far_reset_counter>10 || no_move_reset_counter == 200 ||K[4]<1 || (sqrt(pow((Target_x[step-1]-Position_x[step]),2)+pow((Target_y[step-1]-Position_y[step]),2))>0.5 && step>1))
           {
            //限制條件: 離牆太遠的計數器超過20 或  比 margin laser的限制值小 ... 
             printf(" -- less than safe distance -- \n") ;	
             if (K[4]<1)
             {
              cout<<"速度過慢!!! K[4]<1"<<endl;
             }
             else if (no_move_reset_counter==200)
             {
              cout<<"200步小於2m 速度過慢!!!"<<endl;
             }
             else if(far_reset_counter>10)
             {
              cout<<"角度過大 Trans_error_z= "<<Trans_error_z<<endl;
             }
             current_stop_step=step;					
             remained_step = max_step -current_stop_step ;
           
             printf("current_stop_step === %d\n",current_stop_step);
             
             //重置的歸零動作
             out_y[1]=0;///202011
	           out_y[2]=0;
	           wb_motor_set_velocity(left_wheel , out_y[1]); ///202011
	           wb_motor_set_velocity(right_wheel, out_y[2]);
              
             margin_reset=0;
             break;
            }
            if(robot_x <21 && step==max_step )	
            {
            
             current_stop_step = max_step-(21-robot_x)*10;

             printf(" -- less than safe distance -- \n") ;				

             remained_step = max_step - current_stop_step;

             printf("current_stop_step === %d\n",current_stop_step);
             
             //重置的歸零動作
             out_y[1]=0;///202011
	           out_y[2]=0;
	           wb_motor_set_velocity(left_wheel , out_y[1]); ///202011
	           wb_motor_set_velocity(right_wheel, out_y[2]);
             margin_reset=0;
             break;
            }	
     		
    
           //停止條件  
     //  if(step==max_step ||  ( sqrt(pow((14-Position_x[step]),2)+pow((exp(-0.05*14)*cos(14)-Position_y[step]),2))<=0.3 )  )
       //if(step<=max_step &&  sqrt(pow((21-Position_x[step]),2)+pow((exp(-0.05*21)*cos(21*3)-Position_y[step]),2))<=0.3)
       if(step<=max_step &&  robot_x>21)
       
       {
          current_stop_step= step;  
          remained_step = 0;		
          printf(" 停止，步數用盡 \n") ;      			 
          out_y[1]=0;///202011
	        out_y[2]=0;
	        wb_motor_set_velocity(left_wheel , out_y[1]); ///202011
	        wb_motor_set_velocity(right_wheel, out_y[2]);
          margin_reset=0;
          break;
        }
       
          cout<<"~~~~~~~~~~~~~~~Timer~~~~~~~~~~~~~~~ "<<timer<<endl;

           fflush(stdout);
       }


      evaluate_pop (c_gen, ber, _in_clu, remained_step, current_stop_step) ;  //計算costvalue的前置作業 	
      parent_pop[ber].objv[1] = (parent_pop[ber].objv[1] / total_sucess_step[ber]) + fail_step_record[ber];  
      //計算機器人離牆距離的cost fun.值(即cost value)
      parent_pop[ber].objv[2] = ( 1/(parent_pop[ber].objv[2]/ (total_sucess_step[ber]-mini_step)) ) + fail_step_record[ber];// +0.1*(stop_vel/mini_step)+ fail_step_record[ber];  
      //計算機器人平均移動速度的cost fun.值(即cost value) 
	  //parent_pop[ber].smoothly[1] = (parent_pop[ber].smoothly[1] / total_sucess_step[ber]) + fail_step_record[ber];   
      //計算機器人離牆距離的cost fun.值(即cost value)

	  //parent_pop[ber].objv[1] = parent_pop[ber].objv[1] ;// + 0.5*(parent_pop[ber].smoothly[1]);
	//x  parent_pop[ber].objv[2] = parent_pop[ber].objv[2] ;// + 0.25*(parent_pop[ber].smoothly[1]);
      printf("total_sucess_step[%d]= %f\n",ber,total_sucess_step[ber]);
      printf("fail_step_record[%d]= %f\n",ber,fail_step_record[ber]);
      printf("parent_pop[ber].objv[1]=%f \n",parent_pop[ber].objv[1]);
      cout<<"rms=  "<<rms<<" \t avearge_rms= "<<rms/(double)step<<endl;
      print_gen_obj(c_gen);  //印出親代的cost
     
       //重置點
      robot_postion_reset(0);  
    
     //overlap_degree(ber, _in_clu);  //重疊度的計算
     //fuzzy_set_constraint(ber, _in_clu);  //fuzzy set之間的距離限制

		//parent_pop[ber].objv[1] = parent_pop[ber].objv[1] ;// + 0.5*(parent_pop[ber].smoothly[1]);
	 // parent_pop[ber].objv[2] = parent_pop[ber].objv[2] ;// + 0.25*(parent_pop[ber].smoothly[1]);
    if(ber==popsize)
    {
     break;
    }
  }
////////////////////////  親代主執行區  //////////////////////
/////////////////////////////  親代:重置功能與 cost fun. 值的計算  ///////////////////////////////}

 

 
  sw = 1 ; // 使親代進入非主導排序的條件 
  non_domination(sw , c_gen) ; //非主導排序
  sort_on_crowd(sw , c_gen) ; //排列解的多樣性
  printf("\n gen = %d\n", c_gen); //印出目前是第幾代
  report_best(c_gen,sw) ; //回傳最佳解
 
 
 // 存檔:親代的 costvalue rank 控制器
 
 FILE *fnoise1;  

  fnoise1 = fopen("parent_rank.txt","w"); 	
  for (i=1; i<=popsize; i++)
  {
    fprintf(fnoise1,"%d\t", parent_pop[i].front_rank); //排名
    fprintf(fnoise1, "\n") ;
  } 
  fclose(fnoise1);

FILE *fnoise2;
  fnoise2 = fopen("parent_obj.txt","w"); 	
  for (i=1; i<=popsize; i++)
  {
    for(j=1; j<=nobj;j++)
    {
      fprintf(fnoise2,"%.20lf\t", parent_pop[i].objv[j]) ;  //cost value
    }
     fprintf(fnoise2, "\n") ;
  }
  fclose(fnoise2);


FILE *fnoise3; 
   fnoise3 = fopen("parent_rule.txt","w"); 	
  for (i=1; i<=popsize; i++)
  {   
    for(jj=1; jj<=_rule_number; jj++)
    {
      for(jjj=1; jjj<=_in_varl; jjj++)
      {
        fprintf(fnoise3,"%.10f\t", parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2-1] ); // 中心點
        fprintf(fnoise3,"%.10f\t", parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2] ) ; // 寬度
      }
     fprintf(fnoise3,"%.10f   \t", parent_pop[i].xreal[jj*_mem_length-_out_varl+1] ) ;  // Kp
     fprintf(fnoise3,"%.10f   \t", parent_pop[i].xreal[jj*_mem_length-_out_varl+2] ) ;  // Kd  	
     fprintf(fnoise3,"%.10f   \t", parent_pop[i].xreal[jj*_mem_length-_out_varl+3] ) ;  // Ki			
     fprintf(fnoise3,"%.10f   \t", parent_pop[i].xreal[jj*_mem_length-_out_varl+4] ) ;  // Ki		
     fprintf(fnoise3,"%.10f   \t", parent_pop[i].xreal[jj*_mem_length-_out_varl+5] ) ;  // Ki		
     fprintf(fnoise3,"%.10f   \t", parent_pop[i].xreal[jj*_mem_length-_out_varl+6] ) ;  // Ki		
    }    
    fprintf(fnoise3, "\n") ;
  }
  fclose(fnoise3);
   FILE *fnoise5;
  fnoise5 = fopen("parent_rms.txt","a"); 	
  for(j=1; j<=nobj;j++)
  {
  fprintf(fnoise5,"%d \t%.20lf ", c_gen,parent_pop[1].objv[j]) ;   //cost value
  }
  fprintf(fnoise5, "\n") ;
  fclose(fnoise5);
 }
 //////////////////////////////////////// 子代的生成與計算  ///////////////////////////////////////{ 
  else if(c_gen>1)
  {

    printf("I'm the %d gen!!!!!!!\n",c_gen);
  
    /////////  儲存空間的初始化與歸零  /////////////{
    for (i=1; i<=popsize; i++)
    {
      for (j=1; j<=_max_rule*_mem_length; j++)
      {
         fuzzy_real[i][j] = 0. ;
      }
    }
    /////////  儲存空間的初始化與歸零  //////////////}
   

	double _cg = c_gen ;
	double _mg = ngen ;

	int t_loop = new_row; //做為轉存變數
	kk=1;///202011
	//for (kk=1; kk<=t_loop; kk++) ///202011
	//{
           update_pop(_cg , _mg , kk) ; // 解的更新
	//}

	for (kk=1; kk<=new_row; kk++) 
	{
		gauss_data_limit(kk); // 限制高斯函數資料範圍 
	}

	/////////////////  最小中心點 & 最大中心點 的初始化  ///////////////{

	for(int ss=1; ss<=new_row; ss++ )
	{
		for(int ssss=1; ssss<=_in_varl; ssss++)
		{
			min_m[ss][ssss]=1;  //為了找最小值，所以初始的最小值令為1，即為"最大"的最小值。
			max_m[ss][ssss]=0;  //為了找最大值，所以初始的最大值令為0，即為"最小"的最大值。

			//驗證用
			//printf("min_m[%d][%d]:%f\n",ss,ssss,min_m[ss][ssss]); 
		}
	}
   /////////////////  最小中心點 & 最大中心點 的初始化  ///////////////}  

 
/////////////////////////////  子代:重置功能與 cost fun. 值的計算  ///////////////////////////////{


////////////////////////  子代主執行區  //////////////////////

  ber=0;
  while (ber<new_row) 
  {
    wb_motor_set_velocity(left_wheel , 0);
    wb_motor_set_velocity(right_wheel, 0);
    double timer ;
    ber=ber+1;
    rms=0; 
    child_pop[ber].objv[1]=0; // 將親代的第一個 cost value(牆壁與機器人間的平均距離的函數值)歸零。(沒有五位置訓練要歸零，有的話不用再歸零)
    child_pop[ber].objv[2]=0; // 將親代的第二個 cost value(機器人的平均移動速度的函數值)歸零。(沒有五位置訓練要歸零，有的話不用再歸零)
	//child_pop[ber].smoothly[1]=0; // (沒有五位置訓練要歸零，有的話不用再歸零)
    //total_sucess_step=0; //成功的步數歸零
    fail_step_record[ber]=0;  //如果沒有5位置訓練要先歸零(有的話則不用)
    total_sucess_step[ber]=0; //如果沒有5位置訓練要先歸零(有的話則不用)
    total_fail_step=0;  //失敗的步數歸零
    total_error=0;
   
    target_calculate() ; 
    printf("ber = %d\n",ber);

    int step=0;
    int nod=0;
    int final_counter=0;
     limit_counts =0;
    error=0, turn=0 ,intergral=0 ,derivative=0,lastError=0,error_1=0 ,derivative_1=0 ,lastError_1=0;   
  
      mini_step=0.; 
    robot_postion(step);///202011
    remained_step = 0 ; //殘留步數
    current_stop_step = max_step ;   //max_tune_step:重置點的步數 
    //停止的步數=重置點的步數
    _in_clu = _rule_number;  //rule_number的暫存變數
    int far_reset_counter = 0; //距離過遠而重置的計數器
    double narrow_down = 0.; //?
	  out_y[1]=0;///202011 第一步的第五個輸入要為0
	  out_y[2]=0;
    
   
     l=0;
     timer = 0;
      while (wb_robot_step(100) != -1 ) 
     {  
        timer = wb_robot_get_time();
        step=step+1;     

       //意即，跑到重置點步數前不會跳出迴圈，但同時會累計重置計數器。	
        printf("\n子代執行區!!!!!!!\n");

        fuzzy_in(step) ;  //輸入是雷射數值，只是調整fuzzy的input比例(做正規化)	


        child_pop_cluster(ber,step,c_gen);
		
		 
         Fir_str(in , _rule_number , ber) ; //讀取後件部計算每條rule機發量
         fuzzy(_rule_number , ber); //解模糊產生out_y
         Search_Target(); 
         speed_limit() ; // 速度限制器
          robot_vel[step] =(out_y[1]+out_y[2])/2;
          printf("robot_vel[step]=%lf",robot_vel[step]);
          if(robot_vel[step]<0)
	        {
      	    robot_vel[step]=0;
	
	        }
         data_printf(step, c_gen, ber, _in_clu) ; //印資料	

          ////////////////  馬達輸出  ///////////////////{
          wb_motor_set_velocity(left_wheel , out_y[1]);
          wb_motor_set_velocity(right_wheel, out_y[2]);
          wb_robot_step(200);
          ////////////////  馬達輸出  ///////////////////} 

         
        if(( abs(Trans_error_z)> 3.14159 || limit_counts>=5) && step!=5)  far_reset_counter=11;
          
          else if(abs(Trans_error_z)> stop_max_angle)
          {
             far_reset_counter++;
          } 
          else {
            far_reset_counter--;

            if (far_reset_counter<0) 
             far_reset_counter=0;

          }

           //抓座標位置
           robot_postion(step); 
           printf("far_reset_counter=%d\n",far_reset_counter);
          current_stop_step =step;
           if ( far_reset_counter>10 || no_move_reset_counter == 200 ||K[4]<1 ||(sqrt(pow((Target_x[step-1]-Position_x[step]),2)+pow((Target_y[step-1]-Position_y[step]),2))>0.5 && step>1))
           {
            //限制條件: 離牆太遠的計數器超過20 或  比 margin laser的限制值小 ... 
             printf(" -- less than safe distance -- \n") ;								
             remained_step = max_step -step ;
              if (K[4]<1)
             {
              cout<<"速度過慢!!! K[4]<1"<<endl;
             }
             else if (no_move_reset_counter==200)
             {
              cout<<"200步小於2m 速度過慢!!!"<<endl;
             }
             else if(far_reset_counter>10)
             {
              cout<<"角度過大 Trans_error_z= "<<Trans_error_z<<endl;
             }
             printf("current_stop_step === %d\n",current_stop_step);
             
             //重置的歸零動作
             out_y[1]=0;///202011
	           out_y[2]=0;
	           wb_motor_set_velocity(left_wheel , out_y[1]); ///202011
	           wb_motor_set_velocity(right_wheel, out_y[2]);
             
             margin_reset=0;
             break;
            }	
           if( robot_x<21 && step==max_step)	
            {
            //限制條件: 離牆太遠的計數器超過20 或  比 margin laser的限制值小 ... 
            current_stop_step = max_step-(21-robot_x)*10;
            printf(" -- less than safe distance -- \n") ;
            				
            remained_step = max_step-current_stop_step ;
           
            printf("current_stop_step === %d\n",current_stop_step);
             
             //重置的歸零動作
            out_y[1]=0;///202011
	          out_y[2]=0;
	          wb_motor_set_velocity(left_wheel , out_y[1]); ///202011
	          wb_motor_set_velocity(right_wheel, out_y[2]);
            margin_reset=0;
            break;
            }
     		
    
           //停止條件  
     //  if(step==max_step ||  ( sqrt(pow((14-Position_x[step]),2)+pow((exp(-0.05*14)*cos(14)-Position_y[step]),2))<=0.3 )  )
       //if(step<=max_step && sqrt(pow((21-Position_x[step]),2)+pow((exp(-0.05*21)*cos(21*3)-Position_y[step]),2))<0.3 )
       if(step<=max_step &&  robot_x>21)
    
       {
        current_stop_step=step;
        
          remained_step = 0;		
          printf(" 停止，步數用盡 \n") ;      			 
          out_y[1]=0;///202011
	        out_y[2]=0;
	        wb_motor_set_velocity(left_wheel , out_y[1]); ///202011
	        wb_motor_set_velocity(right_wheel, out_y[2]);
          margin_reset=0;
          break;
        
       }
    

          cout<<"~~~~~~~~~~~~~~~Timer~~~~~~~~~~~~~~~ "<<timer<<endl;

           fflush(stdout);
       }

      
      evaluate_pop (c_gen, ber, _in_clu, remained_step, current_stop_step) ;  //計算costvalue的前置作業 	
     

      child_pop[ber].objv[1] = (child_pop[ber].objv[1] / total_sucess_step[ber]) + fail_step_record[ber];
      //計算機器人離牆距離的cost fun.值(即cost value)
      child_pop[ber].objv[2] = ( 1/((child_pop[ber].objv[2])/ (total_sucess_step[ber]-mini_step))) + fail_step_record[ber];// +0.1*(stop_vel/mini_step)+  fail_step_record[ber];  
       //計算機器人平均移動速度的cost fun.值(即cost value)
	  //child_pop[ber].smoothly[1] = (child_pop[ber].smoothly[1] / total_sucess_step[ber]) + fail_step_record[ber];
	  //計算平滑
       printf("total_sucess_step[%d]= %f\n",ber,total_sucess_step[ber]);
       printf("fail_step_record[%d]= %f\n",ber,fail_step_record[ber]);
       cout<<"rms=  "<<rms<<" \t avearge_rms= "<<rms/(double)step<<endl;
      // printf("sum_error=%f \t,mini_step=%lf \t %lf\n",total_error,mini_step,total_error/mini_step);

	 // child_pop[ber].objv[1] = child_pop[ber].objv[1] ;// + 0.5*(child_pop[ber].smoothly[1]);
	//  child_pop[ber].objv[2] = child_pop[ber].objv[2] ;// + 0.25*(child_pop[ber].smoothly[1]);

    print_gen_obj(c_gen);  //印出子代的cost
     
    //重置點
    robot_postion_reset(0);  
      

      if(ber==new_row)
       {
         break;
       }
    
       
  }
////////////////////////  子代主執行區  //////////////////////
/////////////////////////////  子代:重置功能與 cost fun. 值的計算  ///////////////////////////////}
  merge () ;  //混合
  sw = 3 ;// 使子代進入非主導排序的條件  
  non_domination(sw , c_gen) ; //非主導排序
  sort_on_crowd(sw , c_gen) ;  //排列解的多樣性
  update_newparent() ; //使用參數歸零
  
  //best_solution = report_best(c_gen,sw) ; //回傳最佳解

    report_best(c_gen,sw) ; //回傳最佳解

 // 存檔:親代的 costvalue rank 控制器
 
 FILE *fnoise1;  

  fnoise1 = fopen("parent_rank.txt","w"); 	
  for (i=1; i<=popsize; i++)
  {
    fprintf(fnoise1,"%d\t", parent_pop[i].front_rank); //排名
    fprintf(fnoise1, "\n") ;
  }
  fclose(fnoise1);

FILE *fnoise2;
  fnoise2 = fopen("parent_obj.txt","w"); 	
  for (i=1; i<=popsize; i++)
  {
    for(j=1; j<=nobj;j++)
    {
    fprintf(fnoise2,"%.20lf\t ", parent_pop[i].objv[j]) ;   //cost value
    }
     fprintf(fnoise2, "\n") ;
  }
  fclose(fnoise2);


FILE *fnoise3; 
   fnoise3 = fopen("parent_rule.txt","w"); 	
  for (i=1; i<=popsize; i++)
  {   
    for(jj=1; jj<=_rule_number; jj++)
    {
      for(jjj=1; jjj<=_in_varl; jjj++)
      {
        fprintf(fnoise3,"%.10f\t", parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2-1] ); // 中心點
        fprintf(fnoise3,"%.10f\t", parent_pop[i].xreal[(jj-1)*_mem_length+jjj*2] ) ; // 寬度
      }
     fprintf(fnoise3,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+1] ) ;   // Kp
     fprintf(fnoise3,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+2]  ) ;  //Kd 	
     fprintf(fnoise3,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+3]  ) ;  //Ki
     fprintf(fnoise3,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+4]  ) ;  //Kd 	
     fprintf(fnoise3,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+5]  ) ;  //Ki	
     fprintf(fnoise3,"%.10f   \t",  parent_pop[i].xreal[jj*_mem_length-_out_varl+6]  ) ;  //Ki	
    }    
    fprintf(fnoise3, "\n") ;
  }
  fclose(fnoise3);
  
  		for(int j=1; j<=nobj;j++)
		{
			save_temp_obj(j) ; //存最終fuzzy set解
		}	
  
   // Save all pop
  fpt4 = fopen("..\\..\\generated_file\\all_pop.out","a");
  fprintf(fpt4,"# gen = %d\n ", c_gen);  
  report_pop(fpt4);  // all_pop.out
  fflush(stdout);  // fflush() 就是在「強迫」他將 buffer 中的東西輸出到螢幕.
  fflush(fpt4);
  fclose(fpt4);

  fclose(fnoise3);

  FILE *fnoise5;
  fnoise5 = fopen("parent_rms.txt","a"); 	
  for(j=1; j<=nobj;j++)
  {
  fprintf(fnoise5,"%d \t%.20lf ", c_gen,parent_pop[1].objv[j]) ;   //cost value
  }
  fprintf(fnoise5, "\n") ;
  fclose(fnoise5);

  fflush(stdout);
 } 
  
 //////////////////////////////////////// 子代的生成與計算  ///////////////////////////////////////} 
    
	if(c_gen==ngen)  
	{  
		printf("\n Generations finished, now reporting solutions\n");

		save_final_pareto_front() ; //存最後一代的 front 
		
		for(int j=1; j<=nobj;j++)
		{
			save_final_obj(j) ; //存最終fuzzy set解
		}		
	}
}
///////////////////////////////  主執行序  ///////////////////////////////}


/////////////////////////////     交配    ///////////////////////////////{
// Function to cross two individuals //
//此函式用在"交配與突變"內，即  update_pop 。

void crossover (int kk , int _tour1 , int _tour2)
{
int i;
float rnd,par1,par2,chld1,chld2,betaq,beta,alpha;
float y1,y2,y_max,y_min,expp;
    
rnd=randn(0,1);
    
  if(rnd<=_pcross)
  {
    for(i=1; i<=real_mem; i++)
    {
      par1=parent_pop[_tour1].xreal[i];
      par2=parent_pop[_tour2].xreal[i];
      y_max=max_variable(i);
      y_min=min_variable(i);
      
      rnd=randn(0,1);
      
      if(rnd<=0.5)
      {
        if(fabs(par1 - par2) > 0.000001) 
        {
          if(par2 > par1)
          {
          y2 = par2;
          y1 = par1;
          }
          else
          {
          y2 = par1;
          y1 = par2;
          }
                                           
          if((y1 - y_min) > (y_max - y2))     /*Find beta value*/
          {
          beta = 1 + (2*(y_max - y2)/(y2 - y1));
          }
          else
          {
          beta = 1 + (2*(y1-y_min)/(y2-y1));
          }
                                           
          expp = eta_c + 1.0; 
          beta = 1.0/beta; 
          alpha = 2.0 - pow(beta,expp);
                                           
          if (alpha < 0.0) 
          {
          alpha=0.000001;
          }                                 
          rnd=randn(0,1);                                 
          if (rnd <= 1.0/alpha)
          {
           alpha = alpha*rnd;
           expp = 1.0/(eta_c+1.0);
           betaq = pow(alpha,expp);
          }
          else
          {
          alpha = alpha*rnd;
          alpha = 1.0/(2.0-alpha);
          expp = 1.0/(eta_c+1.0);
         
            if (alpha < 0.0) 
            alpha=0.000001;
         
            if (alpha < 0.0) 
            {
             exit(-1);
            }
                                                  
          betaq = pow(alpha,expp);                                          
          }                                           
       chld1 = 0.5*((y1+y2) - betaq*(y2-y1));
       chld2 = 0.5*((y1+y2) + betaq*(y2-y1));
       }
       else
       {                                  
        betaq = 1.0;
        y1 = par1; y2 = par2;
                                        
         /*Generation two children*/
         chld1 = 0.5*((y1+y2) - betaq*(y2-y1));
         chld2 =  0.5*((y1+y2) + betaq*(y2-y1));
       }
      if (chld1 < y_min) chld1 = y_min;
      if (chld1 > y_max) chld1 = y_max;
      if (chld2 < y_min) chld2 = y_min;
      if (chld2 > y_max) chld2 = y_max;                           
    }
    else
    {
    chld1 = par1;
    chld2 = par2;
    }
   child_pop[kk].xreal[i]=chld1;
   child_pop[kk+1].xreal[i]=chld2;
  }		
 }
}
/////////////////////////////     交配    ///////////////////////////////}

//////////////////////////////  突變   //////////////////////////////////{
//用在解的更新  update_pop

void mutation()
{
  int i,j;
  float rnd,delta,indi,deltaq;
  float y,y_min,y_max,val,xy;
  
  for (i=1; i<=popsize; i++)
  {
    for(j=1; j<=real_mem; j++)
    {
    rnd=randn(0,1);
     if(rnd<= _pmutat) // for each variable find whether to do mutation or not
     {
      y=child_pop[i].xreal[j];
      y_max=max_variable(j);
      y_min=min_variable(j);
                          
       if (y>y_min)
       {
         if((y-y_min) < (y_max-y))
          delta = (y - y_min)/(y_max - y_min);
                                
         else
         delta = (y_max - y)/(y_max - y_min);
                                
         rnd=randn(0,1);
         indi=1.0/(eta_m+1.0);
                                
         if (rnd<=0.5)
         {
          xy = 1.0-delta;
          val = 2*rnd+(1-2*rnd)*(pow(xy,(eta_m+1)));
          deltaq =  pow(val,indi) - 1.0;
         }
         else
         {
          xy = 1.0-delta;
          val = 2.0*(1.0-rnd)+2.0*(rnd-0.5)*(pow(xy,(eta_m+1)));
          deltaq = 1.0 - (pow(val,indi));
         }
          
          y = y + deltaq * (y_max-y_min);
                                 
          if (y < y_min) y=y_min;
          if (y > y_max) y=y_max;
                                 
          child_pop[i].xreal[j]=y;
        }
        else
        {
         xy=randn(0,1);
         child_pop[i].xreal[j]=xy*(y_max-y_min)+y_min;
        }
     
      }
    }
    
    /*for (int k=3; k<=real_mem;k=k+7){
     
     if(child_pop[i].xreal[k+1]>1)  child_pop[i].xreal[k+1]=1;
     if(child_pop[i].xreal[k+2]>1)  child_pop[i].xreal[k+2]=1;
     if(child_pop[i].xreal[k]<max(child_pop[i].xreal[k+1],child_pop[i].xreal[k+2])) child_pop[i].xreal[k]=max(child_pop[i].xreal[k+1],child_pop[i].xreal[k+2]);
     }
    for (int k=6; k<=real_mem;k=k+7){
     if(child_pop[i].xreal[k+1]>5)  child_pop[i].xreal[k+1]=5;
     if(child_pop[i].xreal[k+2]>5)  child_pop[i].xreal[k+2]=5;
     if(child_pop[i].xreal[k]<child_pop[i].xreal[k+1]) child_pop[i].xreal[k]=child_pop[i].xreal[k+1];
     }*/
    /* for (int k=4; k<=real_mem;k=k+9){
            if(child_pop[i].xreal[k+1]<max(child_pop[i].xreal[k+3],child_pop[i].xreal[k+2]))
              child_pop[i].xreal[k+1]=max(child_pop[i].xreal[k+3],child_pop[i].xreal[k+2]);
            if(child_pop[i].xreal[k+4]< child_pop[i].xreal[k+5])
            child_pop[i].xreal[k+4]= child_pop[i].xreal[k+5];
       }
       */
  }
}
//////////////////////////////  突變   //////////////////////////////////}

/////////////////////////////   解的更新   ////////////////////////////////{

void update_pop(double _cg , double _mg , int kk )
{
int _tour1 , _tour2 , jj  ;

  for (jj=1; jj<=popsize; jj=jj+2)
  {	
    do
    {
    _tour1 = tour_selection() ;  // 使用競爭法
    _tour2 = tour_selection() ;  // 使用競爭法
    }
    while (_tour1==_tour2);   
  crossover(jj , _tour1, _tour2) ; //交配
  }	

mutation();  //突變		
}

/////////////////////////////////////////////////////////////////////////}

