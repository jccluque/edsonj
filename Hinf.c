// gcc v1.c -o v1 -lm
// ./v1
// https://www.geeksforgeeks.org/pass-2d-array-parameter-c/
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define PI 3.141592653
#define maxChar 101


typedef struct{	// struct input.txt
    double Uref;
    double Rref;
    double u;
    double r;
    int listlen;
} txtread;


txtread* fileread(char filename[maxChar]);  // READ TXT
void readLLUnsorted(txtread* arr);	     // PLOT READ TXT.
void Hinf(double Ur, double Rr,double u, double r, double *nc, double *nd);
void prodMATRIX(int I, int J, int K, double A[I][K], double B[K][J], double C[I][J]) ;
void VelBody(double ROLL, double PITCH, double YAW, double dotx, double doty, double dotz, double *u, double *v, double *w);
void printMATRIX(int n, int m, double M[n][m]);
void TransMATRIX (int m, int n, double M[m][n], double O[n][m]);
void ToArray (int m, int n, double M[m][n], double A[m*n]);
void addMATRIX(int m,int n, double M[m][n],double N[m][n],double X[m][n]);


double a11, a12, a13, b11, b12, b13, c11, c12, c13;
//double VELX = 0.3,VELY = 0.01,VELZ = 0.1; // PARA PROBAR CALCULO VEL BODY

double A[7][7]={ 	0.3166 ,0.0005,-0.0033 ,2.4029 ,0.0092 ,0.1178 ,0.0025
			,0.0001 ,0.9512 ,0.3681 ,0.0006  ,-0.9006 ,0.0000,-0.0064
 			,-0.0000 ,0.0360 ,0.5546,-0.0007,0.9639,-0.0000 ,0.0009
 			,0.0000  ,-0.0000 ,0.0000 ,0.9992  ,-0.0000 ,0.0000  ,-0.0000
 			,0.0000  ,-0.0000 ,0.0000 ,0.0000 ,0.9998 ,0.0000  ,-0.0000
 			,-2.4926 ,0.0378  ,-0.2380 ,9.1051 ,0.6357  ,-0.5527 ,0.0105
	 		,0.0009  ,-1.6864  ,10.4202 ,0.0224 ,-27.9516 ,0.0008  ,-0.8641};
double B[7][4]={	0.0478,0.0002,0.0480,0.0002
			,0.0000,-0.0072,0.0000,-0.0072
  			,-0.0000,0.0088,0.0000,0.0088
			,0.0400,0.0000,0.0400,0.0000
			,0.0000,0.0200,0.0000,0.0200
			,0.1825,0.0063,0.1822,0.0063
			,0.0005,-0.2871,0.0005,-0.2870};
double C[2][7]={ 	-190.1687,2.8847 ,-18.1579 ,694.6679,48.5009,34.1265,0.8031
			,0.0279  ,-51.4655,318.0072,0.6822  ,-853.0362,0.0251,4.1464};


double D[2][4]={    	13.9269 ,0.4790,13.9043 ,0.4791
	  		,0.0143 ,-8.7620,0.0139  ,-8.7596};
	  		
const double v0=0 ;

double x[7][1]={v0,v0,v0,v0,v0,v0,v0}; // espacio de estados val inicial.
double x_1[7][1]={},x_2[7][1]={};

double y[2][1]={}; //salidas
double y_1[2][1]={},y_2[2][1]={},nc,nd; 




int
main ()
{



    txtread* INDATA= fileread("input.txt");  	//LECTURA DE DATOS

/*  PARA PROBAR
    //readLLUnsorted(INDATA);
    //printf("U[183]:%lf\n",INDATA[183-1].u);

*/

    FILE* fptr = fopen("output.txt", "w");	//ESCRITURA DE DATOS
    
	for(int i=0;i<INDATA[0].listlen;i++)
    	{
	  Hinf(INDATA[i].Uref,INDATA[i].Rref,INDATA[i].u,INDATA[i].r,&nc,&nd); //OPERACION Hinf
	  printf("nc: %f, nd:%f \n",nc,nd);	// DISPLAY
	  fprintf(fptr, "%f %f\n",nc,nd);	// ESCRITURA EN TXT FILE
	}	


	fclose(fptr);				// END TXT WRITE.

	
}
void Hinf(double Ur, double Rr,double u, double r, double *nc, double *nd) 
/*	
	Ur: U reference
	Rr: R reference
	u : velocity X body frame
	r : yaw rate
	
	*nc : Valor en modo comun
	*nd : Valor en modo diferencial	*/
{	

	    
	 // MATRIZ U
	    //r= r * PI / 180; // DEG to RAD
	   double U[4][1]={Ur, Rr, u, r}; // consultar si es YAW o AngRateZ


	 // CALCULO Y
	   prodMATRIX(2, 1, 7, C, x ,y_1); // y_1 = C*x
	   prodMATRIX(2, 1, 4, D, U ,y_2); // y_2 = D*U
	   addMATRIX(2,1,y_1,y_2,y); 	    // y = y_1 + y_2
		//printMATRIX(2,1,y);

	 // CALCULO X
	   prodMATRIX(7, 1, 7, A, x ,x_1); // x_1 = A*x
	   prodMATRIX(7, 1, 4, B, U ,x_2); // x_2 = B*U	   
	   addMATRIX(7,1,x_1,x_2,x);     // X[n+1]= x_1 + x_2

	   double NODES[2]={};
	   ToArray(2,1,y,NODES);

	  // ASIGNACION DE VALORES *u, *v, *w
	  *nc = NODES[0];
	  *nd = NODES[1];



}







void prodMATRIX(int I, int J, int K, double A[I][K], double B[K][J], double C[I][J]) 
// FUNCION PARA PRODUCTO ENTRE 2 MATRICES
// 	MATRIZ A 	* 	MATRIZ B 	= 	MATRIZ C
//  |---SIZE[I,K]---|     |---SIZE[K,J]---| 	   |---SIZE [I,J]---|
{
    int k, n, m;

    /* fill C matrix */
    for (m = 0; m < I; m++) {
    	for (n = 0; n < J; n++) {
    		C[m][n] = 0;
    	}
    }

    /* matrix multiplication */
    for (m = 0; m < I; m++) {
    	for (k = 0; k < K; k++) {
			for (n = 0; n < J; n++) {
				C[m][n] += A[m][k] * B[k][n];
			}
    	}	
    }
}

void printMATRIX(int n, int m, double M[n][m])
// IMPRIME MATRIZ
// MATRIZ M de n ROWS y m COLUMNS

{
  printf ("========PRINT MATRIX========\n");
      for(int i=0; i<n; i++)
      {
        for(int j=0; j<m; j++)
        {
          printf("%f\t",M[i][j]);
	      }     
      printf("\n"); // new line
	}
  printf ("==========================\n");   
}

void VelBody(double ROLL, double PITCH, double YAW, double dotx, double doty, double dotz, double *u, double *v, double *w)
// CALCULA LAS VELOCIDADES DEL CUERPO
// ROLL,PITCH,YAW attitude
// dotx,doty,dotz SON VELOCIDADES NED
// *u,*v,*w VELOCIDADES DEL CUERPO CALCULADAS.
{
	double a11, a12, a13, b11, b12, b13, c11, c12, c13;

	  // DEG to RAD
	  ROLL = ROLL * PI / 180;//VELOCIDADES DEL CUERPO
 	
	  PITCH = PITCH * PI / 180;
	  YAW = YAW * PI / 180;

	  // CALCULO MATRIZ J
	  a11 = cos (YAW) * cos (PITCH);
	  a12 = (cos (YAW) * sin (PITCH) * sin (YAW)) - (sin (YAW) * cos (ROLL));
	  a13 = (sin (YAW) * sin (ROLL) - cos (YAW) * cos (ROLL) * sin (PITCH));

	  b11 = sin (YAW) * cos (PITCH);
	  b12 = (cos (ROLL) * cos (PITCH)) + (sin (ROLL) * sin (PITCH) * sin (YAW));
	  b13 = (cos (ROLL) * sin (PITCH) * sin (YAW) - cos (YAW) * sin (ROLL));

	  c11 = -sin (PITCH);
	  c12 = cos (PITCH) * sin (ROLL);
	  c13 = cos (ROLL) * cos (PITCH);
	  
	  double J1[3][3]={a11,a12,a13,b11,b12,b13,c11,c12,c13}; 	// MATRIZ J1
          
          
          // CALCULO MATRIZ TRANSPUESTA
	  double TJ1[3][3]={}; //matriz transpuesta
	  TransMATRIX (3,3, J1,TJ1);  //TJ1 es la transpuesta.
	  
	  // FORMACION MATRIZ CON VEL NED EN X Y Z
	  double VELNED[3][1]={dotx,doty,dotz};
	  
	  // PRODUCTO DE MATRICES
	  // TJ1 * VELNED;
	  
		  // FUNCION PARA PRODUCTO ENTRE 2 MATRICES
		  // void prodMATRIX(int I, int J, int K, double A[I][K], double B[K][J], double C[I][J]) 
		  // 	MATRIZ A 	* 	MATRIZ B 	= 	MATRIZ C
		  //  |---SIZE[I,K]---|     |---SIZE[K,J]---| 	   |---SIZE [I,J]---|

		  // TJ1 es de [3][3] : 3 row, 3 columns, I=3,K=3
		  // VELNED es de [3][1] : 3 row, 1 column, I=3, J=1
	  	// VELBODY sera [3][1];
          double VELBODY[3][1]={};
	  prodMATRIX(3, 1, 3, TJ1, VELNED, VELBODY) ;
	  
	   double VELBODYarray[3]={};
	 ToArray(3,1,VELBODY,VELBODYarray);

	  // ASIGNACION DE VALORES *u, *v, *w
	  *u = VELBODYarray[0];
	  *v = VELBODYarray[1];
	  *w = VELBODYarray[2];
	  



}

void TransMATRIX (int m, int n, double M[m][n], double O[n][m])
// CALCULO DE MATRIZ TRANSPUESTA
// ENTRA MATRIZ M de m ROWS y n COLUMNS, salida en O[n][m]
{
    int i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < m; j++)
            O[i][j] = M[j][i];
}

void ToArray (int m, int n, double M[m][n], double A[m*n])
// CONVIERTE UN ARRAY MULTIDIMENSIONA (ej. matrices) en un array unidimensional
// se introduce Matriz de "m ROWS" y "n COLUMNS", la matriz M, y la salida A[m*n] elementos
{


  

  int   k=0;
  
  for (int ii=0;ii<m;ii++)
  {
  	for (int jj=0;jj<n;jj++)
  	{
  	 A[k]=M[ii][jj];  
  	 k++;
  	}
  }
}

void addMATRIX(int m,int n, double M[m][n],double N[m][n],double X[m][n])
{
    for (int i=0;i<m;i++)
    {
        for (int j=0;j<n;j++)
        {
            X[i][j]=M[i][j]+N[i][j];
        }
    }
}

txtread* fileread(char filename[maxChar])
{
    int i=0;
    double numUref;
    double numRref;
    double nummin;
    double r;


    txtread* databankList = calloc(0,sizeof(txtread));
    FILE* f = fopen(filename,"r");


    if(f==NULL)
    {
        printf("Error al abrir %s",filename);
        return NULL;
    }


    while (EOF!=fscanf(f,"%lf, %lf, %lf, %lf",&numUref,&numRref,&nummin,&r))
    {
        databankList = (txtread*)realloc(databankList,(i+1)*sizeof(txtread));
        databankList[i].Uref = numUref;
        databankList[i].Rref = numRref;
        databankList[i].u= nummin;
        databankList[i].r = r;
        i++;
    }


    fclose(f);
    databankList[0].listlen = i;
    return databankList;
}



void readLLUnsorted(txtread* arr)
{
    for(int i=0;i<arr[0].listlen;i++)
    {
        printf("Uref: %f Rref: %f u: %f r: %f\n",arr[i].Uref,arr[i].Rref,arr[i].u,arr[i].r);
    }
}

