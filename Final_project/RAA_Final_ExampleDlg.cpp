/*
此智財權屬張文中教授所有，只授權於機器人與自動化應用課程使用，未經書面授權不可拷貝流通。
The copyright is belong to Prof. Wen-Chung Chang, is only applied for use in the course “Robotics and Automation Applications”, and is NOT allowed to be further distributed without written permission.
*/

// RAA_Final_ExampleDlg.cpp : 實作檔
//

#include "stdafx.h"
#include "RAA_Final_Example.h"
#include "RAA_Final_ExampleDlg.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include "afxdialogex.h"
#include "afxcmn.h"
#include "afxwin.h"
//#include <vector> //Toan
#include <opencv2/opencv.hpp> // Toan
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <time.h>
#include <string>

//using namespace std;
using namespace cv;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
#define stdout (&__iob_func()[1])) // Toan

//from PTZ
IplImage* img;
IplImage* img_pro;
IplImage *image2; //成功顯示的圖
int Bmin = 0, Gmin = 0, Rmin = 0;
int Bmax = 255, Gmax = 255, Rmax = 255;
int Bmin4B = 31, Gmin4B = 169, Rmin4B = 0;
int Bmax4B = 255, Gmax4B = 255, Rmax4B = 138;
//origin_theta
float origin_theta = 90;
//int mmovevel = 30;
bool flag2 = false;
int counter = 0;
int counter3 = 0;
double CMobileRobottargetX, CMobileRobottargetY;
CString message;
//執行緒宣告
struct MyThreadInfo{
	HWND hWnd;
}Info1;

// CRAA_Final_ExampleDlg dialog
CRAA_Final_ExampleDlg::CRAA_Final_ExampleDlg(CWnd* pParent /*=NULL*/)
  : CDialogEx(CRAA_Final_ExampleDlg::IDD, pParent)
{
  m_ipText = _T("192.168.10.161"); //攝影機 IP
  m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

struct MyThreadInfo2{
	HWND hWnd;
}Info2;

struct MyThreadInfo3{
	HWND hWnd;
}Info3;

//公用變數
CVitaminCtrl m_vitCtrl1;
IplImage* SourceImg;
double targetX, targetY; //車車中心位置
double targetX_track, targetY_track; //目標物中心位置
int targetSize; //目標物大小
CvCapture* capture;
CRect rect;
CDC *pDC;
HDC hDC;
//CWnd* pwnd;

//顏色濾波用滑桿
CSliderCtrl ctrlSliderBmin;
CSliderCtrl ctrlSliderGmin;
CSliderCtrl ctrlSliderRmin;
CSliderCtrl ctrlSliderBmax;
CSliderCtrl ctrlSliderGmax;
CSliderCtrl ctrlSliderRmax;


bool runTimer = false;
bool runTimer2 = false;
bool runTimer3 = false;
bool flag = false; //決定執行緒與Timer是否動作
//bool flag2 = false; //決定執行緒2與Timer是否動作
bool flag3 = false;

ArRobot robot; //機器人變數
ArTcpConnection con; //機器人連線變數
ArSonarDevice sonar; //機器人超音波感測器
ArSensorReading* sonarReading; //讀取超音波感測器之變數
int SonarNum; //機器人Sonar的數量
int *SonarVal;//機器人Sonar的數值
double maxvel = 0.0; //機器人最大線速度
double mmovevel = 0.0; //機器人移動速度
double mvel = 0.0; //回傳機器人線速度
double mrot = 0.0; //回傳機器人角速度
double mvelL = 0.0; //回傳機器人左輪速度
double mvelR = 0.0; //回傳機器人右輪速度
double px = 0.0; // x position
double py = 0.0; // y position
double theta = 0.0; // angle
int mDist = 0;
int mForDist = 1000;
int mSubDist = 0;
int index = 0; //index of file name of captured images

// Toan: 
Mat mat_srcImg;
// create variable for pixel coordinates
double r1, c1, r2, c2, r3, c3, r4, c4, r5, c5;
double u1,v1, u2,v2, u3,v3, u4,v4, u5,v5; // 2D coordinate in camera frame
double rt1, ct1, rt2, ct2, rt3, ct3, rt4, ct4;
double rt1a, ct1a, rt2a, ct2a, rt3a, ct3a, rt4a, ct4a;
int CHECKERBOARD[4][7];   //4 7  //7 10
#define CHECKERBOARDrow 4
#define CHECKERBOARDcol 7
#define pi 3.14159

// mobile robot parameters
double b=100, s=240; // b: radius of wheel; s: distance between 2 wheels; unit is in mm
// s = 280 shown in AmigoBot document, but in real measurement, s = 240 mm
double wmax = 80; // max velocity used to limit the max velocity, corresponding to 

// camera parameters
double fx = 0.00575; //pixel size in mm
double fy = 0.00575; //pixel size in mm
double fu, fv;
double f_l = 3.45; // focal length in mm

// interaction matrix
int zd = 500; // desired distance in z-axis from camera to the object //520
double z_actual;
double lamda = 50; // proportional gain of the controller; range in (20 --> 250) 100 ok for left hand

// desired 2D coordinate in pixel frame
//double or = 322.39, oc = 193.68;
double or = 352, oc = 240;
double rd1 = 232, cd1 = 112, rd2 = 232, cd2 = 288, rd3 = 410, cd3 = 288, rd4, cd4; // Robot 5 - new - small range
double ud1 = (rd1-or)*fx, vd1 = (cd1-oc)*fy; // the desired feature points in pixel --> convert to mm
double ud2 = (rd2-or)*fx, vd2 = (cd2-oc)*fy;
double ud3 = (rd3-or)*fx, vd3 = (cd3-oc)*fy; 

Mat mat_M = Mat::zeros(6, 2, CV_32FC1);
Mat mat_G = Mat::zeros(6, 6, CV_32FC1);
Mat mat_L = Mat::zeros(6, 6, CV_32FC1); // use 3 corner points: 6 , Toan use :Mat mat_L = Mat::zeros(6, 6, CV_32FC1);
Mat mat_H = Mat::zeros(6, 6, CV_32FC1);
Mat inv_mat_M, inv_mat_H, inv_mat_G, inv_mat_J; // inverse matrix 
Mat mat_vel = Mat::zeros(2, 1, CV_32FC1); // control output w_r and w_l
Mat mat_er = Mat::zeros(6, 1, CV_32FC1); // encoded error between current and desired feature coordinates: 8 for 4 corner points; 2 for only 1 center point
Mat mat_J = Mat::zeros(6, 6, CV_32FC1);
// declaring values of these matrice is written in button "Start thread 2"



CEdit *wEdit_SonarNum; //Sonar的數量
CEdit *wEdit_SonarValue; //Edit Control 顯示Sonar sensor資訊
CEdit *wEdit_ForDist; //Edit Control 顯示Sonar sensor資訊
CSliderCtrl m_ctrlSlider; //滑桿
CEdit *wEdit_vel; //Edit Control 顯示 Vel (mm/s)
CEdit *wEdit_rot; //Edit Control 顯示 Rot (deg/s)
CEdit *wEdit_velL; //Edit Control 顯示 VelL (mm/s)
CEdit *wEdit_velR; //Edit Control 顯示 VelR (mm/s)
CEdit *wEdit_px; //Edit Control 
CEdit *wEdit_py; //Edit Control 
CEdit *wEdit_theta; //Edit Control
CString strvel; //存放Vel的字串
CString strrot; //存放Rot的字串
CString strvelL;//存放VelL的字串
CString strvelR;//存放VelR的字串

CString strpx, strpy, strtheta;//存放VelR的字串


//控制器變數-------
CEdit *wEdit_Speed;
CEdit *wEdit_Command;


//存取檔案
fstream fs;
bool flag1 = true;
// 對 App About 使用 CAboutDlg 對話方塊

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();
	
// 對話方塊資料
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支援

// 程式碼實作
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	

	DDX_Control(pDX, IDC_SLIDER1, ctrlSliderBmin);  
	DDX_Control(pDX, IDC_SLIDER2, ctrlSliderBmax);

	DDX_Control(pDX, IDC_SLIDER3, ctrlSliderGmin);
	DDX_Control(pDX, IDC_SLIDER4, ctrlSliderGmax);

	DDX_Control(pDX, IDC_SLIDER5, ctrlSliderRmin);
	DDX_Control(pDX, IDC_SLIDER6, ctrlSliderRmax);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CRAA_Final_ExampleDlg 對話方塊




//CRAA_Final_ExampleDlg::CRAA_Final_ExampleDlg(CWnd* pParent /*=NULL*/)
	//: CDialogEx(CRAA_Final_ExampleDlg::IDD, pParent)
//{
	//m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
//}

void CRAA_Final_ExampleDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_ipTEXT, m_ipText);
	DDX_Control(pDX, IDC_AXISMEDIACONTROL1, m_AMC);
	//
	DDX_Control(pDX, IDC_VITAMINCTRL1, m_vitCtrl1);

	DDX_Control(pDX, IDC_SLIDER1, m_ctrlSlider);
	DDX_Control(pDX, IDC_SLIDER2, ctrlSliderBmin);
	DDX_Control(pDX, IDC_SLIDER3, ctrlSliderGmin);
	DDX_Control(pDX, IDC_SLIDER4, ctrlSliderRmin);
	DDX_Control(pDX, IDC_SLIDER5, ctrlSliderBmax);
	DDX_Control(pDX, IDC_SLIDER6, ctrlSliderGmax);
	DDX_Control(pDX, IDC_SLIDER7, ctrlSliderRmax);
}

BEGIN_MESSAGE_MAP(CRAA_Final_ExampleDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(btnConnectRobot, &CRAA_Final_ExampleDlg::OnBnClickedbtnconnectrobot)
	ON_BN_CLICKED(btnDisconnectRobot, &CRAA_Final_ExampleDlg::OnBnClickedbtndisconnectrobot)
	ON_BN_CLICKED(btnGetSonarNum, &CRAA_Final_ExampleDlg::OnBnClickedbtngetsonarnum)
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER1, &CRAA_Final_ExampleDlg::OnNMCustomdrawSlider1)
	ON_BN_CLICKED(btnThread, &CRAA_Final_ExampleDlg::OnBnClickedbtnthread)
	ON_BN_CLICKED(btnThread2, &CRAA_Final_ExampleDlg::OnBnClickedbtnthread2)
	ON_BN_CLICKED(btnSaveInfo, &CRAA_Final_ExampleDlg::OnBnClickedbtnsaveinfo)
	ON_BN_CLICKED(btnConnectCam, &CRAA_Final_ExampleDlg::OnBnClickedbtnconnectcam)
ON_BN_CLICKED(btnCaptureImg, &CRAA_Final_ExampleDlg::OnBnClickedbtncaptureimg)
ON_WM_TIMER()
ON_BN_CLICKED(btnForward, &CRAA_Final_ExampleDlg::OnBnClickedbtnforward)
ON_BN_CLICKED(btnStop, &CRAA_Final_ExampleDlg::OnBnClickedbtnstop)
ON_BN_CLICKED(btnBackward, &CRAA_Final_ExampleDlg::OnBnClickedbtnbackward)
ON_BN_CLICKED(btnLeft, &CRAA_Final_ExampleDlg::OnBnClickedbtnleft)
ON_BN_CLICKED(btnRight, &CRAA_Final_ExampleDlg::OnBnClickedbtnright)
//ON_BN_CLICKED(IDC_BUTTON8, &CRAA_Final_ExampleDlg::OnBnClickedButton8)
ON_BN_CLICKED(IDC_BUTTON1, &CRAA_Final_ExampleDlg::OnBnClickedButton1)
ON_BN_CLICKED(IDC_BUTTON2, &CRAA_Final_ExampleDlg::OnBnClickedButton2)
ON_BN_CLICKED(IDC_BUTTON3, &CRAA_Final_ExampleDlg::OnBnClickedButton3)
ON_STN_CLICKED(picShowImg, &CRAA_Final_ExampleDlg::OnStnClickedpicshowimg)
ON_BN_CLICKED(IDC_BUTTON4, &CRAA_Final_ExampleDlg::OnBnClickedButton4)
ON_BN_CLICKED(IDC_BUTTON5, &CRAA_Final_ExampleDlg::OnBnClickedButton5)
ON_COMMAND(IDD_RAA_FINAL_EXAMPLE_DIALOG, &CRAA_Final_ExampleDlg::OnIddRaaFinalExampleDialog)
ON_EN_CHANGE(IDC_EDIT1, &CRAA_Final_ExampleDlg::OnEnChangeEdit1)
ON_EN_CHANGE(IDC_ipTEXT, &CRAA_Final_ExampleDlg::OnEnChangeiptext)
ON_BN_CLICKED(IDC_BUTTON9, &CRAA_Final_ExampleDlg::OnBnClickedButton9)
ON_BN_CLICKED(IDC_BUTTON10, &CRAA_Final_ExampleDlg::OnBnClickedButton10)
ON_BN_CLICKED(IDC_BUTTON11, &CRAA_Final_ExampleDlg::OnBnClickedButton11)
ON_BN_CLICKED(IDC_BUTTON12, &CRAA_Final_ExampleDlg::OnBnClickedButton12)
ON_BN_CLICKED(IDC_BUTTON13, &CRAA_Final_ExampleDlg::OnBnClickedButton13)
ON_EN_CHANGE(txtVel, &CRAA_Final_ExampleDlg::OnEnChangetxtvel)
ON_EN_CHANGE(txtMaxVel, &CRAA_Final_ExampleDlg::OnEnChangetxtmaxvel)
ON_EN_CHANGE(txtDist, &CRAA_Final_ExampleDlg::OnEnChangetxtdist)
END_MESSAGE_MAP()


// CRAA_Final_ExampleDlg 訊息處理常式

BOOL CRAA_Final_ExampleDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 將 [關於...] 功能表加入系統功能表。

	// IDM_ABOUTBOX 必須在系統命令範圍之中。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 設定此對話方塊的圖示。當應用程式的主視窗不是對話方塊時，
	// 框架會自動從事此作業
	SetIcon(m_hIcon, TRUE);			// 設定大圖示
	SetIcon(m_hIcon, FALSE);		// 設定小圖示

	// TODO: 在此加入額外的初始設定
	maxvel = 80;
	CString str("");
	str.Format("%.f", maxvel);
	SetDlgItemText(txtMaxVel, _T(str)); //設定IDC_MaxVel初始值
	m_ctrlSlider.SetRange(0,500); //設定Slider的範圍
	m_ctrlSlider.SetPos(100); //設定Slider的初始位置

	ctrlSliderBmin.SetRange(0,255);
	ctrlSliderGmin.SetRange(0,255);
	ctrlSliderRmin.SetRange(0,255);
	ctrlSliderBmax.SetRange(0,255);
	ctrlSliderGmax.SetRange(0,255);
	ctrlSliderRmax.SetRange(0,255);

	ctrlSliderBmin.SetPos(58);
	ctrlSliderGmin.SetPos(31);
	ctrlSliderRmin.SetPos(138);
	ctrlSliderBmax.SetPos(215);
	ctrlSliderGmax.SetPos(166);
	ctrlSliderRmax.SetPos(255);

	SetDlgItemText(txtDist, _T("300")); //設定IDC_MaxVel初始值
	SetDlgItemText(txtSubDist, _T("500")); //設定IDC_MaxVel初始值

	GetDlgItem(txtCamIP)->SetWindowText("192.168.10.161");
	GetDlgItem(txtCamPort)->SetWindowText("80");
	GetDlgItem(txtCamID)->SetWindowText("root");
	GetDlgItem(txtCamPWD)->SetWindowText("");

	//GetDlgItem(IDC_PicCtrl)->MoveWindow(825,125,350,200);


	return TRUE;  // 傳回 TRUE，除非您對控制項設定焦點
}

void CRAA_Final_ExampleDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

void CRAA_Final_ExampleDlg::ShowImg(IplImage* img, UINT ID)
{
	CDC *pDC = GetDlgItem(ID)->GetDC();
	HDC hDC = pDC->GetSafeHdc();
	CRect rect;
	GetDlgItem(ID)->GetClientRect(&rect);
	CvvImage cimg;
	cimg.CopyOf(img);
	cimg.DrawToHDC(hDC, &rect);
	ReleaseDC(pDC);
}

// 如果將最小化按鈕加入您的對話方塊，您需要下列的程式碼，
// 以便繪製圖示。對於使用文件/檢視模式的 MFC 應用程式，
// 框架會自動完成此作業。

void CRAA_Final_ExampleDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 繪製的裝置內容

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 將圖示置中於用戶端矩形
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 描繪圖示
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 當使用者拖曳最小化視窗時，
// 系統呼叫這個功能取得游標顯示。
HCURSOR CRAA_Final_ExampleDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CRAA_Final_ExampleDlg::OnBnClickedbtnconnectrobot()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	//初始化一些全域變數
	Aria::init();

	//設定機器人連線
	con.setPort("192.168.10.136", 8101); //設定機器人IP 
	robot.setDeviceConnection(&con); //與機器人連線
	if (!robot.blockingConnect())
	{
		printf("Could not connect to robot... Exiting.");
		Aria::shutdown();
	}

	robot.addRangeDevice(&sonar); //啟動Sonar

	robot.runAsync(true);

	robot.lock();
	robot.comInt(ArCommands::SOUNDTOG, 1);		// enables the buzzer sounds.
	robot.enableMotors();
	robot.enableSonar(); // enables the sonar.
	robot.requestEncoderPackets();// Starts a continuous stream of encoder packets.

	//robot.setAbsoluteMaxTransVel(50); //設定機器人最大線速度，機器人線速度設定後直到下次設定才會修改
	robot.unlock();
}


void CRAA_Final_ExampleDlg::OnBnClickedbtndisconnectrobot()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	robot.stop();
	m_vitCtrl1.Disconnect();
	Aria::exit(0); //離線
}


void CRAA_Final_ExampleDlg::OnBnClickedbtngetsonarnum()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	wEdit_SonarNum = (CEdit*)GetDlgItem(txtSonarNum);
	wEdit_SonarValue = (CEdit*)GetDlgItem(txtSonarValue);
	CString str2;

	robot.lock();
	SonarNum = robot.getNumSonar(); //取得Sonar sensor總數
	SonarVal = new int[SonarNum];

	for (int i = 0; i < SonarNum; i++)
	{
		sonarReading = robot.getSonarReading(i); //讀取第i個Sonar讀取數值
		CString temp;
		temp.Format(_T("read %d = %d\r\n"), i, sonarReading->getRange()); //將Sonar讀取資訊放入字串中
		str2 += temp;
	}
	robot.unlock();

	CString str;
	str.Format(_T("%d"), SonarNum);
	wEdit_SonarNum->SetWindowText(str); //顯示Sonar總數
	wEdit_SonarValue->SetWindowText(str2); //顯示Sonar讀取資訊
}


void CRAA_Final_ExampleDlg::OnNMCustomdrawSlider1(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO: 在此加入控制項告知處理常式程式碼
	*pResult = 0;

	CString str("");
	str.Format("%d", m_ctrlSlider.GetPos()); //取得滑桿數值
	SetDlgItemText(txtNowVel, str);
	mmovevel = atoi(str); //將文字轉為double

	if (mmovevel >= maxvel) //若前進速度大於最大速度
	{
		mmovevel = maxvel; //設定最大速度為前進速度
	}
}

UINT MyThreadFun(LPVOID LParam) //設定執行緒之動作
{
	//clock_t a,b;
	//ofstream timeout("timercount.txt");
	while (flag)
	{
		//a = clock();
		CString str2;

		robot.lock();
		//取得機器人估測(X,Y)座標 getX(), getY()
		mvel = robot.getVel(); //讀取機器人線速度(mm/s)
		mrot = robot.getRotVel(); //讀取機器人角速度(deg/s)
		mvelR = robot.getRightVel(); //讀取左輪線速度(mm/s)
		mvelL = robot.getLeftVel(); //讀取機器人右輪線速度(mm/s)
		//get pose of mobile robot
		px = robot.getX();
		py = robot.getY();
		theta = robot.getTh();
		//
		for (int i = 0; i < SonarNum; i++)
		{
			sonarReading = robot.getSonarReading(i); //讀取第i個Sonar資訊
			CString temp;
			SonarVal[i] = sonarReading->getRange();
			temp.Format(_T("read %d = %d\r\n"), i, sonarReading->getRange()); //取出第i個Sonar偵測障礙物距離
			str2 += temp;
		}
		robot.unlock();

		mForDist = min(mForDist, min(SonarVal[2], SonarVal[3]));

		// show values of velocities
		strvel.Format(_T("%.2f"), mvel);
		wEdit_vel->SetWindowText(strvel); //設定Edit Control之顯示文字
		strrot.Format(_T("%.2f"), mrot);
		wEdit_rot->SetWindowText(strrot); //設定Edit Control之顯示文字
		strvelL.Format(_T("%.2f"), mvelR);
		wEdit_velR->SetWindowText(strvelL); //設定Edit Control之顯示文字
		strvelR.Format(_T("%.2f"), mvelL);
		wEdit_velL->SetWindowText(strvelR); //設定Edit Control之顯示文字
		// show values of pose of mobile robot
		strpx.Format(_T("%.2f"), px);
		wEdit_px->SetWindowText(strpx); 
		strpy.Format(_T("%.2f"), py);
		wEdit_py->SetWindowText(strpy); 
		strtheta.Format(_T("%.2f"), theta);
		wEdit_theta->SetWindowText(strtheta); 

		// show distances from sonar sensors
		wEdit_SonarValue->SetWindowText(str2); //設定Edit Control之顯示文字
		CString strtmp;
		strtmp.Format(_T("%d"), mForDist);
		wEdit_ForDist->SetWindowText(strtmp);
		z_actual = (SonarVal[2] + SonarVal[3])/2;
		//b = clock();
		//timeout << b - a << endl; 
	}
	return 0;
}

void CRAA_Final_ExampleDlg::OnBnClickedbtnthread()
{
	//指定變數指向控制元件
	wEdit_vel = (CEdit*)GetDlgItem(txtVel);
	wEdit_rot = (CEdit*)GetDlgItem(txtVelRot);
	wEdit_velR = (CEdit*)GetDlgItem(txtVelR);
	wEdit_velL = (CEdit*)GetDlgItem(txtVelL);
	wEdit_px = (CEdit*)GetDlgItem(txtpx); 
	wEdit_py = (CEdit*)GetDlgItem(txtpy);
	wEdit_theta = (CEdit*)GetDlgItem(txttheta);

	wEdit_ForDist = (CEdit*)GetDlgItem(txtMinDist);

	Info1.hWnd = this->m_hWnd;

	// TODO: 在此加入控制項告知處理常式程式碼
	if (flag)
	{
		flag = false;
	}
	else
	{
		flag = true;
		AfxBeginThread(MyThreadFun, (LPVOID)&Info1); //啟動執行緒
	}
}

UINT MyThreadFun2(LPVOID LParam) //設定執行緒之動作, 控制器
{
	return 0;
}
void CRAA_Final_ExampleDlg::OnBnClickedbtnthread2()
{
	
}


void CRAA_Final_ExampleDlg::OnBnClickedbtnsaveinfo()
{
	//char filename[80];
	//// save poses of mobile robot
	//index++;
	//sprintf(filename,"D:\Poses_%d.txt",index);
	//fs.open(filename, ios::out);
	//fs << px << " " << py << " " << theta <<"\n";
	//fs.close();
	//// save images
	////index++;
	////char filename[80];
	//// TODO: Add your control notification handler code here
	//sprintf(filename,"D:\img_%d.bmp",index);
	//cvSaveImage(filename, SourceImg);


	char filename[80];
	// save poses of mobile robot
	index++;
	string tmpfile("Poses.txt");
    fstream file;
	file.open(tmpfile, std::ios_base::app | std::ios_base::in);
    if (file.is_open())
        file << px << " " << py << " " << theta <<"\n";
	// save images
	sprintf(filename,"D:\img_%d.bmp",index);
	//cvSaveImage(filename, SourceImg);	
	imwrite(filename, mat_srcImg);
	fs.close();
}




void CRAA_Final_ExampleDlg::OnBnClickedbtnconnectcam()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	int ii = m_vitCtrl1.GetControlStatus();
	if (m_vitCtrl1.GetControlStatus() != 0 &&
		m_vitCtrl1.GetControlStatus() != 3 &&
		m_vitCtrl1.GetControlStatus() != 5) // sam add connection broken
	{
		m_vitCtrl1.Disconnect();
		return;
	}

	CString strServIP, strRootPwd, strUserName, strTemp;

	GetDlgItem(txtCamIP)->GetWindowText(strServIP);
	GetDlgItem(txtCamPort)->GetWindowText(strTemp);
	GetDlgItem(txtCamID)->GetWindowText(strUserName);
	GetDlgItem(txtCamPWD)->GetWindowText(strRootPwd);

	m_vitCtrl1.SetPassword(strRootPwd);
	m_vitCtrl1.SetUserName(strUserName);
	m_vitCtrl1.SetRemoteIPAddr(strServIP);
	long lPort = _tstoi(LPCTSTR(strTemp));
	if (lPort <= 0 || lPort > 65535)
		lPort = 80;
	m_vitCtrl1.SetHttpPort(lPort);

	m_vitCtrl1.Connect();

	GetDlgItem(txtCamIP)->EnableWindow(FALSE);
	GetDlgItem(txtCamPort)->EnableWindow(FALSE);
	GetDlgItem(txtCamID)->EnableWindow(FALSE);
	GetDlgItem(txtCamPWD)->EnableWindow(FALSE);

	GetDlgItem(btnConnectCam)->SetWindowText("Stop");

	m_dwDecodedAudioSize = 0;
	m_dwDecodedVideoSize = 0;
}



UINT MyThreadFun3(LPVOID LParam) //擷取影像
{
	//CRAA_Final_ExampleDlg* pDlg = (CRAA_Final_ExampleDlg*) LParam; 
	while(flag3)
	{
		//pDlg->OnTimer(2);
		IplImage* tempImg = NULL;
		VARIANT vData, vInfo;

		long lRet = m_vitCtrl1.GetSnapshot(4, &vData, &vInfo);

		unsigned char* buff;
		SafeArrayAccessData(vData.parray, (void**)&buff);

		if (tempImg)
			cvReleaseImage(&tempImg);
		tempImg = cvCreateImage(cvSize(320, 200), IPL_DEPTH_8U, 3);

		memcpy(tempImg->imageData, buff, 320 * 200 * 3);
		SafeArrayUnaccessData(vData.parray);
		VariantClear(&vData);
		VariantClear(&vInfo);
	
		SourceImg = tempImg;
		cvReleaseImage(&tempImg);
		//ShowImg(SourceImg, picShowImg);
	}
	return 0;
}



void CRAA_Final_ExampleDlg::OnBnClickedbtncaptureimg()
{

	// TODO: 在此加入控制項告知處理常式程式碼
	if(runTimer)
	{
		runTimer = false;
		fs.close(); //關閉檔案
		KillTimer(1); //關閉Timer
		//KillTimer(2); //關閉Timer
	}
	else
	{
		runTimer = true;
		m_Second = 100; //Timer每100ms執行一次
		SetTimer(1, m_Second, NULL); //啟動Timer
		//SetTimer(2, m_Second, NULL); //啟動Timer
	}

	//int count=0;
	//int R = 0, G = 0, B = 0;//存放單位pixel的各顏色強度值
	////int black = 0;//存放單位pixel的灰階強度值
	//IplImage *image = NULL;

	//// TODO: 在此加入控制項告知處理常式程式碼
	//VARIANT vData, vInfo;
	//
	//if (image2) cvReleaseImage(&image2);
	//image2 = cvLoadImage("08.jpg", 1); //成功顯示的圖

	//long lRet = m_vitCtrl1.GetSnapshot(4, &vData, &vInfo);

	//unsigned char* buff;
	//SafeArrayAccessData (vData.parray, (void**)&buff);

	//if(image) cvReleaseImage(&image);
	//
	//image = cvCreateImage(cvSize(320,200), IPL_DEPTH_8U, 3);

	//memcpy(image->imageData, buff, 320*200*3);

	//cvNamedWindow("Show Image",0);
	//cvShowImage("Show Image",image);


	//for (int i = 0; i < image->width; i++)
	//{
	//	for (int j = 0; j < image->height; j++)
	//	{
	//		B = ((uchar *)(image->imageData + j*image->widthStep))[i*image->nChannels + 0];	// get Blue
	//		G = ((uchar *)(image->imageData + j*image->widthStep))[i*image->nChannels + 1];	// get Green
	//		R = ((uchar *)(image->imageData + j*image->widthStep))[i*image->nChannels + 2];	// get Red
	//		if ((B < 50) && (G <50) && (R <50))
	//		{
	//			count++;
	//		}

	//	}
	//}
	//if (count == ((image->width)*(image->height)))
	//{
	//	//sing and circle
	//	for(int i=0;i<1200;i++)//右轉三次--->旋轉
	//	{
	//		robot.lock();
	//		robot.setVel2(100, -100);//R
	//		robot.unlock();
	//	}
	//	count = 0;
	//	cvNamedWindow("Show Image",0);
	//	cvShowImage("Show Image",image2);//顯示成功圖片
	//}

	//if (flag3)
	//{
	//	flag3 = false;
	//}
	//else
	//{
	//	flag3 = true;
	//	AfxBeginThread(MyThreadFun3, (LPVOID)&Info3); //啟動執行緒
	//}
}


void CRAA_Final_ExampleDlg::GetImage(IplImage* img)
{
	
	
	
}


void CRAA_Final_ExampleDlg::OnTimer(UINT_PTR nIDEvent)
{
	//clock_t a,b,c,d;
	long bufferSize;
	byte* buf;
	int W = 704; //704
	int H = 480;
	CString str("");
	IplImage* rangeImg;
	IplImage* rangeImg_track;
	IplImage* tempImg;
	IplImage* tempImg2;
	Mat  canny_output, canny_output_track, mat_dstImg, mat_dstImg_track;
	vector<vector<Point> > contours, contours_track;
	vector<Vec4i> hierarchy, hierarchy_track;
	CvRect rect;
	CvPoint center;
	CString ctrlURL;
	CvFont font;
	//CDialog::OnTimer(nIDEvent);
	ofstream filterOut("filter.txt");
	//ofstream MposeOut("Mpose.txt");
	double tracking_theta, mobileRobot_theta;
	double hypotenuse, hypotenusea;

	switch(nIDEvent)
	{
		case 1:
			//a = clock();
			VARIANT var;
			
			cvNamedWindow("axis",1);
			VariantInit(&var);
			m_AMC.GetCurrentImage(1,&var,&bufferSize);
			buf = var.pbVal;
			buf += sizeof(BITMAPINFOHEADER);
			img = cvCreateImage(cvSize(W, H),8,3);
			memcpy(img->imageData,buf,img->imageSize);

			cvFlip(img,img,0);
			if(!img)
				break;
			cvShowImage("axis",img);
			VariantClear(&var);

			//SourceImg = cvCloneImage(img);
			//cvCopyImage(img, SourceImg);
			//SourceImg = cvCopyImage(img);
			//img.copyTo(SourceImg);
			SourceImg = img;
			//cvReleaseImage(&img);
		break;

		//影像處理
		case 2:
		{
			if (!SourceImg)
				break;
			img_pro = cvCreateImage(cvGetSize(SourceImg), 8, 3);
			cvCvtColor(SourceImg, img_pro, CV_BGR2YCrCb); //YCRCB 空間轉換 亮度，顏色分布。
			// 取得滑桿數值
			str.Format("%d", ctrlSliderBmin.GetPos()); //取得滑桿數值
			SetDlgItemText(IDC_STATICBmin, str);
			Bmin = atoi(str);
			str.Format("%d", ctrlSliderGmin.GetPos()); //取得滑桿數值
			SetDlgItemText(IDC_STATICGmin, str);
			Gmin = atoi(str);
			str.Format("%d", ctrlSliderRmin.GetPos()); //取得滑桿數值
			SetDlgItemText(IDC_STATICRmin, str);
			Rmin = atoi(str);
			str.Format("%d", ctrlSliderBmax.GetPos()); //取得滑桿數值
			SetDlgItemText(IDC_STATICBmax, str);
			Bmax = atoi(str);
			str.Format("%d", ctrlSliderGmax.GetPos()); //取得滑桿數值
			SetDlgItemText(IDC_STATICGmax, str);
			Gmax = atoi(str);
			str.Format("%d", ctrlSliderRmax.GetPos()); //取得滑桿數值
			SetDlgItemText(IDC_STATICGmax, str);
			Rmax = atoi(str);

			

			// contour 輪廓
			//處理藍色色塊
			//cvErode(rangeImg, rangeImg, 0, 1); // cvErode(輸入圖像,輸出圖像,null為3x3,運作次數)
			//cvDilate(rangeImg, rangeImg, 0, 1);
			//tempImg = cvCloneImage(rangeImg);
			//canny_output = cv::cvarrToMat(rangeImg);
			//findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
			//處理紅色色塊
			//cvErode(rangeImg_track, rangeImg_track, 0, 1); // cvErode(輸入圖像,輸出圖像,null為3x3,運作次數)
			//cvDilate(rangeImg_track, rangeImg_track, 0, 1);
			////tempImg = cvCloneImage(rangeImg_track);
			//cvAdd(tempImg, rangeImg_track, tempImg);
			//canny_output_track = cv::cvarrToMat(rangeImg_track);
			//findContours(canny_output_track, contours_track, hierarchy_track, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			Mat mat_ImgY, mat_rangeImg, mat_rangeImg_track;
			mat_srcImg = cvarrToMat(SourceImg);
			cvtColor(mat_srcImg, mat_ImgY, CV_BGR2YCrCb);
			inRange(mat_ImgY, Scalar(Bmin, Gmin, Rmin), Scalar(Bmax, Gmax, Rmax), mat_rangeImg);
			inRange(mat_ImgY, Scalar(Bmin4B, Gmin4B, Rmin4B), Scalar(Bmax4B, Gmax4B, Rmax4B), mat_rangeImg_track);
			//opening
			erode(mat_rangeImg, mat_rangeImg, Mat(), Point(-1, -1), 1, 1, 1); // Mat() means the kernerl size is in default: 3 x 3
			dilate(mat_rangeImg, mat_rangeImg, Mat(), Point(-1, -1), 1, 1, 1);
			//closing
			dilate(mat_rangeImg, mat_rangeImg, Mat(), Point(-1, -1), 1, 1, 1);
			erode(mat_rangeImg, mat_rangeImg, Mat(), Point(-1, -1), 1, 1, 1); // Mat() means the kernerl size is in default: 3 x 3
			//opening
			erode(mat_rangeImg_track, mat_rangeImg_track, Mat(), Point(-1, -1), 1, 1, 1); // Mat() means the kernerl size is in default: 3 x 3
			dilate(mat_rangeImg_track, mat_rangeImg_track, Mat(), Point(-1, -1), 1, 1, 1);
			//closing
			dilate(mat_rangeImg_track, mat_rangeImg_track, Mat(), Point(-1, -1), 1, 1, 1);
			erode(mat_rangeImg_track, mat_rangeImg_track, Mat(), Point(-1, -1), 1, 1, 1); // Mat() means the kernerl size is in default: 3 x 3
			mat_rangeImg.convertTo(mat_dstImg, CV_8UC3);//(3 channels)
			findContours(mat_dstImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
			int font_face = cv::FONT_HERSHEY_COMPLEX;
			double font_scale = 0.5; // font size
			int thickness = 1; // font thickness

			//track
			mat_rangeImg_track.convertTo(mat_dstImg_track, CV_8UC3);//(3 channels)
			findContours(mat_dstImg_track, contours_track, hierarchy_track, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
			int font_face_track = cv::FONT_HERSHEY_COMPLEX;
			double font_scale_track = 0.5; // font size
			int thickness_track = 1; // font thickness
			//Undone Task2:找到紅色色塊的四個點--------------------------------------------------------------------------------------------------------------------------
			for (int i = 0; i < contours_track.size(); i++)//輪廓的數量
			{
				//依照色板的角度以及距離設定
				if (contourArea(contours_track[i]) < 5000 && contourArea(contours_track[i]) > 500)
				{
					rect = boundingRect(Mat(contours_track[i]));
					RotatedRect Rrect = minAreaRect(Mat(contours_track[i]));
					tracking_theta = Rrect.angle;
					if (tracking_theta < -45)
						tracking_theta += 90;
					if (-90 < tracking_theta < 90)
					{
						//紅色色塊偏移
						targetX_track = rect.x - 75 * cos(tracking_theta * pi / 180) + rect.width / 2.0;
						targetY_track = rect.y - 75 * sin(tracking_theta * pi / 180) + rect.height / 2.0;
					}

					hypotenuse = sqrt(pow(rect.width / 2.0, 2) + pow(rect.height / 2.0, 2)) * (cos(theta * pi / 180) / 5 + 0.8);
					double h = sqrt(pow((rect.width / 2.0) + 100, 2) + pow(rect.height / 2.0, 2));

					//計算紅色色塊的四個點，從左上為(rt1,ct1)，逆時針開始依序計算(rt2,ct2)、(rt3,ct3)、(rt4,ct4)
					rt1 = targetX_track- cos((45 + tracking_theta) * pi / 180) * hypotenuse; ct1 = targetY_track - sin((45 + tracking_theta) * pi / 180) * hypotenuse;
					rt2 = targetX_track- cos((45 - tracking_theta) * pi / 180) * hypotenuse; ct2 = targetY_track + sin((45 - tracking_theta) * pi / 180) * hypotenuse;
					rt3 = targetX_track+ cos((45 + tracking_theta) * pi / 180) * hypotenuse; ct3 = targetY_track + sin((45 + tracking_theta) * pi / 180) * hypotenuse;
					rt4 = targetX_track+ cos((45 - tracking_theta) * pi / 180) * hypotenuse; ct4 = targetY_track - sin((45 - tracking_theta) * pi / 180) * hypotenuse;
					

					// 紅色色塊4個點畫圓標記該點座標
					circle(mat_srcImg, cvPoint(rt1, ct1), 5, Scalar(0, 0, 255), 3, CV_AA, 0);
					circle(mat_srcImg, cvPoint(rt2, ct2), 5, Scalar(0, 0, 255), 3, CV_AA, 0);

					circle(mat_srcImg, cvPoint(rt3, ct3), 5, Scalar(0, 0, 255), 3, CV_AA, 0);

					circle(mat_srcImg, cvPoint(rt4, ct4), 5, Scalar(0, 0, 255), 3, CV_AA, 0);
					
					//...cvPoint(rt2, ct2)...
					// ...cvPoint(rt3, ct3)...
					// ...cvPoint(rt4, ct4)...

					//write the coordinates
					stringstream temp; // in camera frame
					temp << 1 << "(" << (int)rt1 << "," << (int)ct1 << ")";
					cv::putText(mat_srcImg, temp.str(), Point(rt1, ct1 - 15), font_face_track, font_scale_track, cv::Scalar(0, 255, 0), thickness_track, 8, 0);
					temp.str("");
					temp << 2 << "(" << (int)rt2 << "," << (int)ct2 << ")";
					cv::putText(mat_srcImg, temp.str(), Point(rt2, ct2 + 15), font_face_track, font_scale_track, cv::Scalar(0, 255, 0), thickness_track, 8, 0);
					temp.str("");
					temp << 3 << "(" << (int)rt3 << "," << (int)ct3 << ")";
					cv::putText(mat_srcImg, temp.str(), Point(rt3, ct3 + 15), font_face_track, font_scale_track, cv::Scalar(0, 255, 0), thickness_track, 8, 0);
					temp.str("");
					temp << 4 << "(" << (int)rt4 << "," << (int)ct4 << ")";
					cv::putText(mat_srcImg, temp.str(), Point(rt4, ct4 - 15), font_face_track, font_scale_track, cv::Scalar(0, 255, 0), thickness_track, 8, 0);
					temp << 5 << "(" << tracking_theta << ")";
					cv::putText(mat_srcImg, temp.str(), Point(rt4, ct4 - 50), font_face_track, font_scale_track, cv::Scalar(0, 255, 0), thickness_track, 8, 0);
					//Because the y-axis direction is opposite to our designated coordinate, we need to convert it. 
					ct1 *= -1;
					ct2 *= -1;
					ct3 *= -1;
					ct4 *= -1;
					break;
				}
			}
			//part1
			//Undone Task1:找到藍色色塊的四個點--------------------------------------------------------------------------------------------------------------------------
			for (int i = 0; i < contours.size(); i++)//輪廓的數量
			{
				//依照色板的角度以及距離設定
				if (contourArea(contours[i]) > 500 && contourArea(contours[i]) < 5000)
				{
					rect = boundingRect(Mat(contours[i]));
					RotatedRect Rrect = minAreaRect(Mat(contours[i]));
					//藍色色塊中心點 (矩形左上角為(rect.x,rect.y))
					//targetX =  ;
					//targetY = ;
					
					targetX = rect.x +  rect.width / 2.0;
					targetY = rect.y + rect.height / 2.0;
					double theta_target = atan((targetY - targetY_track) / (targetX - targetY_track));
			
					hypotenuse = sqrt(pow(rect.width / 2.0, 2) + pow(rect.height / 2.0, 2)) * (cos(theta * pi / 180) / 5 + 0.8);
					CString debug_angle;
					
					mobileRobot_theta = Rrect.angle;
					if (mobileRobot_theta < -45)
						mobileRobot_theta += 90;
					
					//計算藍色色塊的四個點，從左上為(r1,c1)，逆時針開始依序計算(r2,c2)、(r3,c3)、(r4,c4)
					r1 = targetX - cos((45 + mobileRobot_theta) * pi / 180) * hypotenuse; c1 = targetY - sin((45 + mobileRobot_theta) * pi / 180) * hypotenuse;
					r2 = targetX - cos((45 - mobileRobot_theta) * pi / 180) * hypotenuse; c2 = targetY + sin((45 - mobileRobot_theta) * pi / 180) * hypotenuse;
					r3 = targetX + cos((45 + mobileRobot_theta) * pi / 180) * hypotenuse; c3 = targetY + sin((45 + mobileRobot_theta) * pi / 180) * hypotenuse;
					r4 = targetX + cos((45 - mobileRobot_theta) * pi / 180) * hypotenuse; c4 = targetY - sin((45 - mobileRobot_theta) * pi / 180) * hypotenuse;
					
					theta = -mobileRobot_theta;
					debug_angle.Format(_T("theta : %f \n "), theta);
					OutputDebugString(debug_angle);
					// 藍色色塊4個點畫圓標記該點座標
					circle(mat_srcImg, cvPoint(r1, c1), 5, Scalar(255, 0, 0), 3, CV_AA, 0);
					circle(mat_srcImg, cvPoint(r2, c2), 5, Scalar(255, 0, 0), 3, CV_AA, 0);

					circle(mat_srcImg, cvPoint(r3, c3), 5, Scalar(255, 0, 0), 3, CV_AA, 0);

					circle(mat_srcImg, cvPoint(r4, c4), 5, Scalar(255, 0, 0), 3, CV_AA, 0);
					
					//write the coordinates
					stringstream temp; // in camera frame
					temp << 1 << "(" << (int)r1 << "," << (int)c1 << ")";
					cv::putText(mat_srcImg, temp.str(), Point(r1, c1), font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);
					temp.str("");
					temp << 2 << "(" << (int)r2 << "," << (int)c2 << ")";
					cv::putText(mat_srcImg, temp.str(), Point(r2, c2), font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);
					temp.str("");
					temp << 3 << "(" << (int)r3 << "," << (int)c3 << ")";
					cv::putText(mat_srcImg, temp.str(), Point(r3, c3 + 15), font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);
					temp.str("");
					temp << 4 << "(" << (int)r4 << "," << (int)c4 << ")";
					cv::putText(mat_srcImg, temp.str(), Point(r4, c4 + 15), font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);

					//Because the y-axis direction is opposite to our designated coordinate, we need to convert it. 
					c1 *= -1;
					c2 *= -1;
					c3 *= -1;
					c4 *= -1;
					break;
				}
			}
			//------------------------------------------------------------------------------------------------------------------------
			


			
			//藍色變數:  contours, targetX, targetY
			//紅色變數:  contours_track, targetX_track, targetY_track
			//共用變數:  tempImg, rect 
			//
			// for, if: 判斷面積大小(500-5000)，找出藍色中心點
			// cvCircle:畫圓
			// cvInitFont, cvPutText:顯示文字
			//
			// for, if: 判斷面積大小(500-5000)，找出紅色中心點並位移
			// cvCircle:畫圓
			// cvInitFont, cvPutText:顯示文字
			//
			//--------------------------------------------------------------------------------------------------------------------------

			cv::imshow("output", mat_srcImg);
			cvReleaseImage(&img);
			cvReleaseImage(&img_pro);
			break;
		}
		//控制器
		case 3:

			double speedLeft;
			double speedRight;
			double mid_robotX , mid_robotY ;
			double mid_targetX, mid_targetY;
			static clock_t last_time = clock();

			clock_t current_time = clock();
			double dt = fabs(double(current_time - last_time)) / CLOCKS_PER_SEC; 
			static double current_theta = 0;
			last_time = current_time;

			//if turn left getRotVel is positive
			current_theta += dt * robot.getRotVel();
			current_theta %= 360;
			mid_targetX = (rt1 + rt2 + rt3 + rt4) / 4;
			mid_targetY = (ct1 + ct2 + ct3 + ct4) / 4;

			mid_robotX = (r1 + r2 + r3 + r4)  / 4;
			mid_robotY = (c1 + c2 + c3 + c4) / 4;
			
			double theta_target = atan2(mid_targetY - mid_robotY, mid_targetX - mid_robotX) * 180 / pi;
	
			
			message.Format(_T("target_theta: %f\n"), (theta_target) );
			OutputDebugString(message);
			message.Format(_T("theta_target - origin_theta: %f\n"), (theta_target - origin_theta) );
			OutputDebugString(message);

			// front 0 left 90 back 180 right -90
			double diff_angle = ((theta_target - origin_theta) - current_theta );
			message.Format(_T("theta_target - theta: %f\n"), (theta_target - theta));
			OutputDebugString(message);
			
			message.Format(_T("origin_theta: %f\n"), origin_theta);
			OutputDebugString(message);


			if (fabs(diff_angle) <= 10)
			{
				current_theta = theta_target;
				message.Format(_T("rotate complete: % f\n"));
				OutputDebugString(message);
				double distance = fabs(sqrt(pow(mid_targetY - mid_robotY, 2) + pow(mid_targetX - mid_robotX, 2)));

				if ( distance <= 50){
					message.Format(_T("ALL complete: % f\n"));
					OutputDebugString(message);
				}
				else
				{
					speedRight = sin(theta_target);
					speedLeft = sin(theta_target);
				}
				
			}
			else {
				speedLeft = cos(theta_target);
				speedRight = cos(180-(theta_target));
				speedRight *= (theta_target - current_theta) / theta_target;
				speedLeft *= (theta_target - current_theta) / theta_target;
				
				

			}
			speedLeft *= 50;
			speedRight *= 50;
			message.Format(_T("Left, Right: %f"), speedLeft, speedRight);
			OutputDebugString(message);
			robot.lock();
			robot.setVel2(speedLeft, speedRight); // w_l and w_r
			robot.unlock();
		break;
	}
}


void CRAA_Final_ExampleDlg::OnBnClickedbtnforward()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	robot.lock();
	robot.setVel2(mmovevel, mmovevel);
	robot.unlock();
}


void CRAA_Final_ExampleDlg::OnBnClickedbtnstop()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	robot.lock();
	robot.setVel2(0, 0);
	robot.unlock();
}


void CRAA_Final_ExampleDlg::OnBnClickedbtnbackward()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	robot.lock();
	robot.setVel2(-mmovevel, -mmovevel);
	robot.unlock();
}


void CRAA_Final_ExampleDlg::OnBnClickedbtnleft()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	robot.lock();
	robot.setVel2(-mmovevel, mmovevel);
	robot.unlock();
}


void CRAA_Final_ExampleDlg::OnBnClickedbtnright()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	robot.lock();
	robot.setVel2(mmovevel, -mmovevel);
	robot.unlock();
}



void CRAA_Final_ExampleDlg::OnBnClickedButton1()
{
	// TODO: 在此加入控制項告知處理常式程式碼
		if(runTimer2)
	{
		runTimer2 = false;
		fs.close(); //關閉檔案
		KillTimer(2); //關閉Timer
	}
	else
	{
		runTimer2 = true;
		m_Second = 100; //Timer每100ms執行一次
		SetTimer(2, m_Second, NULL); //啟動Timer
	}
}


void CRAA_Final_ExampleDlg::OnBnClickedButton2()
{
	//fs.close(); // stop saving poses of the mobile robot
}


void CRAA_Final_ExampleDlg::OnBnClickedButton3()
{
	index++;
	char filename[80];
	// TODO: Add your control notification handler code here
	sprintf(filename,"D:\img_%d.bmp",index);
	cvSaveImage(filename, SourceImg);
}


void CRAA_Final_ExampleDlg::OnStnClickedpicshowimg()
{
	// TODO: Add your control notification handler code here
}


void CRAA_Final_ExampleDlg::OnBnClickedButton4()
{
	// TODO: Add your control notification handler code here
	try
  {
    CString ctrlURL, presetURL, mediaURL;
    this->UpdateData();
    ctrlURL.Format(_T("http://%s/axis-cgi/com/ptz.cgi"), m_ipText);
    mediaURL.Format(_T("http://%s/axis-cgi/mjpg/video.cgi"), m_ipText);
    presetURL.Format(_T("http://%s/axis-cgi/param.cgi?usergroup=anonymous&action=list&group=PTZ.Preset.P0"), m_ipText);
    // Firmware version 4
    //presetURL.Format(_T("http://%s/axis-cgi/view/param.cgi?action=list&group=PTZ.Preset.P0"), m_ipText);

    //Stops possible streams
    m_AMC.Stop();
	
    // Set the PTZ control properties
    m_AMC.put_PTZControlURL(ctrlURL);
    m_AMC.put_UIMode((CString)"ptz-absolute");

    // Enable PTZ-position presets from AMC context menu
    m_AMC.put_PTZPresetURL(presetURL);

    // Enable joystick support
    m_AMC.put_EnableJoystick(TRUE);

    // Enable area zoom
    m_AMC.put_EnableAreaZoom(TRUE);

    // Enable one-click-zoom
    //m_AMC.put_OneClickZoom(TRUE);

    // Set overlay settings
    m_AMC.put_EnableOverlays(TRUE);
    m_AMC.put_ClientOverlay(AMC_OVERLAY_CROSSHAIR |
                            AMC_OVERLAY_VECTOR |
                            AMC_OVERLAY_ZOOM);

    // Show the status bar and the tool bar in the AXIS Media Control
    m_AMC.put_ShowStatusBar(true);
    m_AMC.put_ShowToolbar(true);
    m_AMC.put_StretchToFit(true);
    m_AMC.put_EnableContextMenu(true);
    m_AMC.put_ToolbarConfiguration((CString)"default,-mute,-volume,+ptz");

    // Set the media URL and the media type
    m_AMC.put_MediaURL(mediaURL);
    m_AMC.Play();
  }
  catch (COleDispatchException *e)
  {      
    MessageBox(e->m_strDescription);
  }
}

UINT MyThreadFun4(LPVOID LParam) //設定執行緒之動作, 控制器
{
	return 0;
}

void CRAA_Final_ExampleDlg::OnBnClickedButton5()
{
}


void CRAA_Final_ExampleDlg::OnIddRaaFinalExampleDialog()
{
	// TODO: Add your command handler code here
}


void CRAA_Final_ExampleDlg::OnEnChangeEdit1()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
}


void CRAA_Final_ExampleDlg::OnEnChangeiptext()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
}

void CRAA_Final_ExampleDlg::OnBnClickedButton9()
{
	// TODO: Add your control notification handler code here
}


void CRAA_Final_ExampleDlg::OnBnClickedButton10()
{
	// TODO: Add your control notification handler code here
	//AllocConsole();
	//freopen("CONOUT$", "w", stdout;
	//cout << targetSize<< ", X:"<< targetX << ", Y:" << targetY << endl;
	//CMobileRobottargetX = targetX - 352; 
	//CMobileRobottargetY = -(targetY - 240);
	rd1 = r1; rd2 = r2; rd3 = r3; cd1 = c1; cd2 = c2; cd3 = c3;
	//rd1 = r1; rd4 = r4; rd3 = r3; cd1 = c1; cd4 = c4; cd3 = c3;
	//rd1 = rt1+100; rd2 = rt2+100; rd3 = rt3+100; cd1 = ct1; cd2 = ct2; cd3 = ct3;
}


void CRAA_Final_ExampleDlg::OnBnClickedButton11()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	if(runTimer3)
	{
		runTimer3 = false;
		fs.close(); //關閉檔案
		KillTimer(3); //關閉Timer
	}
	else
	{
		runTimer3 = true;
		m_Second = 100; //Timer每100ms執行一次
		SetTimer(3, m_Second, NULL); //啟動Timer
	}
}


void CRAA_Final_ExampleDlg::OnBnClickedButton12()
{
	// TODO: Add your control notification handler code here
	robot.lock();
	robot.setVel2(90, 100); // w_l and w_r
	robot.unlock();
}
UINT MyThreadFun5(LPVOID LParam)
{
	return 0;
}

void CRAA_Final_ExampleDlg::OnBnClickedButton13()
{
	// TODO: Add your control notification handler code here
	if (counter3==1)
	{
		flag3 = false;
		counter3 = 0;
		AfxBeginThread(MyThreadFun5, (LPVOID)&Info3); //啟動執行緒
		robot.lock();
		robot.setVel2(0, 0);
		robot.unlock();
	}
	else
	{
		flag3 = true;
		counter3 = 1;
		AfxBeginThread(MyThreadFun5, (LPVOID)&Info3); //啟動執行緒
	}
}


void CRAA_Final_ExampleDlg::OnEnChangetxtvel()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
}


void CRAA_Final_ExampleDlg::OnEnChangetxtmaxvel()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
}


void CRAA_Final_ExampleDlg::OnEnChangetxtdist()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
}
