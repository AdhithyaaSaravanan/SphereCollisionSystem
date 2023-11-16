
#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "NGLScene.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_gl = new NGLScene(this);

    ui->s_mainWindowGridLayout->addWidget(m_gl, 0, 0, 6, 3);

    // set position
    connect(ui->posX,SIGNAL(valueChanged(double)),m_gl,SLOT(setXPosition(double)));
    connect(ui->posY,SIGNAL(valueChanged(double)),m_gl,SLOT(setYPosition(double)));
    connect(ui->posZ,SIGNAL(valueChanged(double)),m_gl,SLOT(setZPosition(double)));

    // set velocity
    connect(ui->velX,SIGNAL(valueChanged(double)),m_gl,SLOT(setXVelocity(double)));
    connect(ui->velY,SIGNAL(valueChanged(double)),m_gl,SLOT(setYVelocity(double)));
    connect(ui->velZ,SIGNAL(valueChanged(double)),m_gl,SLOT(setZVelocity(double)));

    // colour
    connect(ui->r,SIGNAL(valueChanged(int)),m_gl,SLOT(setColR(int)));
    connect(ui->g,SIGNAL(valueChanged(int)),m_gl,SLOT(setColG(int)));
    connect(ui->b,SIGNAL(valueChanged(int)),m_gl,SLOT(setColB(int)));


    // physics
    connect(ui->Mass,SIGNAL(valueChanged(double)),m_gl,SLOT(setMass(double)));
    connect(ui->frictionValue,SIGNAL(valueChanged(double)),m_gl,SLOT(setFriction(double)));
    connect(ui->energyLossValue,SIGNAL(valueChanged(double)),m_gl,SLOT(setEnergyLoss(double)));
    connect(ui->gravityValue,SIGNAL(valueChanged(double)),m_gl,SLOT(setGravity(double)));
    connect(ui->Radius, SIGNAL(valueChanged(double)), m_gl, SLOT(setRadius(double)));

    // sim controls
    connect(ui->minMass,SIGNAL(valueChanged(double)),m_gl,SLOT(setMinMass(double)));
    connect(ui->maxMass,SIGNAL(valueChanged(double)),m_gl,SLOT(setMaxMass(double)));

    connect(ui->minRadius,SIGNAL(valueChanged(double)),m_gl,SLOT(setMinRadius(double)));
    connect(ui->maxRadius,SIGNAL(valueChanged(double)),m_gl,SLOT(setMaxRadius(double)));

    connect(ui->numSpheres,SIGNAL(valueChanged(int)),m_gl,SLOT(setNumSpheres(int)));

    // bbox
    connect(ui->bboxSize,SIGNAL(valueChanged(double)),m_gl,SLOT(setBBoxSize(double)));

    // buttons 
    connect(ui->addSphere,SIGNAL(clicked()),m_gl,SLOT(addSphere()));
    connect(ui->simControls,SIGNAL(clicked()),m_gl,SLOT(startSim()));
    connect(ui->randomise,SIGNAL(clicked()),m_gl,SLOT(randomise()));
    connect(ui->reset,SIGNAL(clicked()),m_gl,SLOT(resetScene()));
}

MainWindow::~MainWindow()
{
    delete ui;
}


