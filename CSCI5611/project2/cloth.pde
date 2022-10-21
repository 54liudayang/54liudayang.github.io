import java.util.ArrayList;
import java.util.Locale;

import com.thomasdiewald.pixelflow.java.DwPixelFlow;
import com.thomasdiewald.pixelflow.java.imageprocessing.filter.DwFilter;
import com.thomasdiewald.pixelflow.java.softbodydynamics.DwPhysics;
import com.thomasdiewald.pixelflow.java.softbodydynamics.constraint.DwSpringConstraint;
import com.thomasdiewald.pixelflow.java.softbodydynamics.particle.DwParticle;
import com.thomasdiewald.pixelflow.java.softbodydynamics.particle.DwParticle3D;
import com.thomasdiewald.pixelflow.java.softbodydynamics.softbody.DwSoftBall3D;
import com.thomasdiewald.pixelflow.java.softbodydynamics.softbody.DwSoftBody3D;
import com.thomasdiewald.pixelflow.java.softbodydynamics.softbody.DwSoftGrid3D;
import com.thomasdiewald.pixelflow.java.utils.DwCoordinateTransform;
import com.thomasdiewald.pixelflow.java.utils.DwStrokeStyle;


import peasy.CameraState;
import peasy.PeasyCam;
import processing.core.PApplet;
import processing.core.PFont;
import processing.core.PShape;
import processing.opengl.PGraphics2D;
import processing.opengl.PGraphics3D;

 
 
  // Everything can collide with everything and also be destroyed (RMB).
  // 

  //
  // Controls:
  // LMB: drag particles
  // MMB: drag + fix particles to a location
  // RMB: disable springs, to deform objects
  //
  // ALT + LMB: Camera ROTATE
  // ALT + MMB: Camera PAN
  // ALT + RMB: Camera ZOOM
  //
  
  int viewport_w = 1280;
  int viewport_h = 720;
  int viewport_x = 230;
  int viewport_y = 0;
  
  int gui_w = 200;
  int gui_x = viewport_w - gui_w;
  int gui_y = 0;
  
  // main library context
  DwPixelFlow context;
  
  DwPhysics.Param param_physics = new DwPhysics.Param();
  
  // // physics simulation object
  DwPhysics<DwParticle3D> physics = new DwPhysics<DwParticle3D>(param_physics);
  
  // cloth objects
  DwSoftGrid3D cloth = new DwSoftGrid3D();
  DwSoftBall3D ball  = new DwSoftBall3D();

  // list, that wills store the softbody objects (cloths, cubes, balls, ...)
  ArrayList<DwSoftBody3D> softbodies = new ArrayList<DwSoftBody3D>();
  
  // particle parameters
  DwParticle.Param param_cloth_particle = new DwParticle.Param();
  DwParticle.Param param_ball_particle  = new DwParticle.Param();
  
  // spring parameters
  DwSpringConstraint.Param param_cloth_spring = new DwSpringConstraint.Param();
  DwSpringConstraint.Param param_ball_spring  = new DwSpringConstraint.Param();
  

  // camera
  PeasyCam peasycam;
  CameraState cam_state_0;
  
  // cloth texture
  PGraphics2D texture;
  
  // global states
  int BACKGROUND_COLOR = 92;
  
  // 0 ... default: particles, spring
  // 1 ... tension
  
  
  // entities to display
 
  boolean DISPLAY_MESH           = true;
 
  boolean DISPLAY_SPRINGS_STRUCT = true;
  boolean DISPLAY_SPRINGS_SHEAR  = true;
  boolean DISPLAY_SPRINGS_BEND   = true;
  
  boolean UPDATE_PHYSICS         = true;
  
  // first thing to do, inside draw()
  boolean NEED_REBUILD = false;
  

  public void settings(){
    size(viewport_w, viewport_h, P3D); 
    smooth(8);
  }
  
  
  public void setup() {
    surface.setLocation(viewport_x, viewport_y);

    // main library context
    context = new DwPixelFlow(this);
    context.print();
    context.printGL();
    

    ////////////////////////////////////////////////////////////////////////////
    // PARAMETER settings
    // ... to control behavior of particles, springs, etc...
    ////////////////////////////////////////////////////////////////////////////
    
    // physics world parameters
    int cs = 1500;
    param_physics.GRAVITY = new float[]{ 0, 0, -0.1f};
    param_physics.bounds  = new float[]{ -cs, -cs, 0, +cs, +cs, +cs };
    param_physics.iterations_collisions = 2;
    param_physics.iterations_springs    = 8;
    
    // particle parameters (for simulation)
    param_cloth_particle.DAMP_BOUNDS    = 0.49999f;
    param_cloth_particle.DAMP_COLLISION = 0.99999f;
    param_cloth_particle.DAMP_VELOCITY  = 0.99991f; 
    
    param_ball_particle.DAMP_BOUNDS    = 0.89999f;
    param_ball_particle.DAMP_COLLISION = 0.99999f;
    param_ball_particle.DAMP_VELOCITY  = 0.0000f; 
   
    // spring parameters (for simulation)
    param_cloth_spring.damp_dec = 0.999999f;
    param_cloth_spring.damp_inc = 0.059999f;
    
    param_ball_spring.damp_dec = 0.999999999f;
    param_ball_spring.damp_inc = 0.9999999f;
    
    // soft-body parameters (for building)
    cloth.CREATE_STRUCT_SPRINGS = true;
    cloth.CREATE_SHEAR_SPRINGS  = true;
    cloth.CREATE_BEND_SPRINGS   = true;
    cloth.bend_spring_mode      = 0;
    cloth.bend_spring_dist      = 2;

    ball.CREATE_STRUCT_SPRINGS = true;
    ball.CREATE_SHEAR_SPRINGS  = true;
    ball.CREATE_BEND_SPRINGS   = true;
    ball.bend_spring_mode      = 0;
    ball.bend_spring_dist      = 10;

    // softbodies
    createBodies();
    
    // modelview/projection
    createCam();
    
    
   

    frameRate(600);
  }
  
  
  public void createBodies(){
    
    // first thing to do!
    physics.reset();
    
    int nodex_x, nodes_y, nodes_z, nodes_r;
    int nodes_start_x, nodes_start_y, nodes_start_z;
    int ball_subdivisions, ball_radius;
    float r,g,b,s;
    
    boolean particles_volume = false;
    
    
    // add to global list
    softbodies.clear();
    softbodies.add(cloth);
    softbodies.add(ball);
    
    // set some common things, like collision behavior
    for(DwSoftBody3D body : softbodies){
      body.self_collisions = true;
      body.collision_radius_scale = 1f;
    }
   
    
    ///////////////////// CLOTH ////////////////////////////////////////////////
    nodex_x = 40;
    nodes_y = 40;
    nodes_z = 1;
    nodes_r = 10;
    nodes_start_x = 0;
    nodes_start_y = 0;
    nodes_start_z = nodes_y * nodes_r*2-200;
    r = 255;
    g = 200;
    b = 100;
    s = 1f;
    cloth.texture_XYp = texture;
    cloth.setMaterialColor(color(r  ,g  ,b  ));
    cloth.setParticleColor(color(r*s,g*s,b*s));
    cloth.setParam(param_cloth_particle);
    cloth.setParam(param_cloth_spring);
    cloth.create(physics, nodex_x, nodes_y, nodes_z, nodes_r, nodes_start_x, nodes_start_y, nodes_start_z);
    cloth.createShapeParticles(this, particles_volume);
    cloth.getNode(              0, 0, 0).enable(false, false, false);
    cloth.getNode(cloth.nodes_x-1, 0, 0).enable(false, false, false);


    //////////////////// BALL //////////////////////////////////////////////////
    ball_subdivisions = 3;
    ball_radius = 140;
    nodes_start_x = 400;
    nodes_start_y = 500;
    nodes_start_z = 400;
    r = 100;
    g = 150;
    b = 0;
    s = 1f;
    ball.setMaterialColor(color(r  ,g  ,b  ));
    ball.setParticleColor(color(r*s,g*s,b*s));
    ball.setParam(param_ball_particle);
    ball.setParam(param_ball_spring);
    ball.create(physics, ball_subdivisions, ball_radius, nodes_start_x, nodes_start_y, nodes_start_z);
    ball.createShapeParticles(this, particles_volume);
    
  }

  
  //////////////////////////////////////////////////////////////////////////////
  // draw()
  //////////////////////////////////////////////////////////////////////////////
  
  public void draw() {
    
    if(NEED_REBUILD){
      createBodies();
      NEED_REBUILD = false;
    }

    // add additional forces, e.g. Wind, ...
    int particles_count = physics.getParticlesCount();
    DwParticle[] particles = physics.getParticles();
    if(APPLY_WIND){
      float[] wind = new float[3];
      for(int i = 0; i < particles_count; i++){
        wind[1] = noise(i) *0.5f;
        particles[i].addForce(wind);
      }
    }

    // update interactions, like editing particle positions, deleting springs, etc...
    updateMouseInteractions();
    
    // update physics simulation
    if(UPDATE_PHYSICS){
      physics.update(1);
    }
    
    // update softbody surface normals
    for(DwSoftBody3D body : softbodies){
      body.computeNormals();
    }
    

    ////////////////////////////////////////////////////////////////////////////
    // RENDER this madness
    ////////////////////////////////////////////////////////////////////////////
    background(BACKGROUND_COLOR);
    
    // disable peasycam-interaction while we edit the model
    peasycam.setActive(MOVE_CAM);
    
    
    
    strokeWeight(2);



    
    // lights, materials
    // lights();
    pointLight(220, 180, 140, -1000, -1000, -100);
    ambientLight(96, 96, 96);
    directionalLight(210, 210, 210, -1, -1.5f, -2);
    lightFalloff(1.0f, 0.001f, 0.0f);
    lightSpecular(255, 0, 0);
    specular(255, 0, 0);
    shininess(5);
      
   
    if(DISPLAY_MESH){
      for(DwSoftBody3D body : softbodies){
        body.createShapeMesh(this.g);
      }
    }
  
    if(DISPLAY_MESH){
      for(DwSoftBody3D body : softbodies){
        body.displayMesh(this.g);
      }
    }
   

    // info
    int NUM_SPRINGS   = physics.getSpringCount();
    int NUM_PARTICLES = physics.getParticlesCount();
    String txt_fps = String.format(getClass().getName()+ "   [particles %d]   [springs %d]   [frame %d]   [fps %6.2f]", NUM_PARTICLES, NUM_SPRINGS, frameCount, frameRate);
    surface.setTitle(txt_fps);
  }
  
  
  
  //////////////////////////////////////////////////////////////////////////////
  // User Interaction
  //////////////////////////////////////////////////////////////////////////////
  
  // coordinate transformation: world <-> screen
  // for interaction
  final DwCoordinateTransform transform = new DwCoordinateTransform();
  
  float[] mouse_world  = new float[4];
  float[] mouse_screen = new float[4];

  ParticleTransform pnearest;
  
  class ParticleTransform{
    DwParticle particle = null;
    float[]      screen = new float[4];

    public ParticleTransform(DwParticle particle, float[] screen){
      set(particle, screen);
    }
    public void set(DwParticle particle, float[] screen){
      this.particle = particle;
      this.screen[0] = screen[0];
      this.screen[1] = screen[1];
      this.screen[2] = screen[2];
      this.screen[3] = screen[3];
    }
  }
  

  void findNearestParticle(float mx, float my, float radius){
    int particles_count = physics.getParticlesCount();
    DwParticle[] particles = physics.getParticles();
    
    float radius_sq = radius * radius;
    float dd_min = radius_sq;
    float dz_min = 1;
    float[] screen = new float[4];
    pnearest = null;
    
    // transform Particles: world -> screen
    for(int i = 0; i < particles_count; i++){
      DwParticle3D pa = (DwParticle3D) particles[i];
      transform.transformToScreen(pa, screen);
      float dx = screen[0] - mx;
      float dy = screen[1] - my;
      float dz = screen[2];
      float dd_sq = dx*dx + dy*dy;
      if(dd_min > dd_sq){
        if(dz_min > dz){
          dz_min = dz;
          dd_min = dd_sq;
          pnearest = new ParticleTransform(pa, screen);
        }
      }
    }  
  }
  
  ArrayList<DwParticle> particles_within_radius = new ArrayList<DwParticle>();
  void findParticlesWithinRadius(float mx, float my, float radius){
    int particles_count = physics.getParticlesCount();
    DwParticle[] particles = physics.getParticles();
    
    float dd_min = radius * radius;
    particles_within_radius.clear();
    float[] screen = new float[4];
    
    // transform Particles: world -> screen
    for(int i = 0; i < particles_count; i++){
      DwParticle3D pa = (DwParticle3D) particles[i];
      transform.transformToScreen(pa, screen);
      float dx = screen[0] - mx;
      float dy = screen[1] - my;
      float dd_sq = dx*dx + dy*dy;
      if(dd_min > dd_sq){
        particles_within_radius.add(pa);
      }
    }  
  }
  
  
  public void updateMouseInteractions(){
    
    // update transformation stuff
    transform.useCurrentTransformationMatrix((PGraphics3D) this.g);
    
    // deleting springs/constraints between particles
    if(DELETE_SPRINGS){
      findParticlesWithinRadius(mouseX, mouseY, DELETE_RADIUS);
      for(DwParticle particle : particles_within_radius){
        particle.enableAllSprings(false);
        particle.collision_group = physics.getNewCollisionGroupId();
        particle.rad_collision = particle.rad;
        particle.all_springs_deactivated = true;
      }
    } 
    
    if(!MOVE_PARTICLE){
      findNearestParticle(mouseX, mouseY, SNAP_RADIUS);
      SNAP_PARTICLE = pnearest != null;
    }
    
    if(SNAP_PARTICLE){
      mouse_screen[0] = mouseX;
      mouse_screen[1] = mouseY;
      mouse_screen[2] = pnearest.screen[2];
      transform.screenToWorld(mouse_screen, mouse_world);
      
      if(MOVE_PARTICLE){
        pnearest.particle.enable(false, false, false);
        pnearest.particle.moveTo(mouse_world, 0.1f);
      }
    }
  }
  
  
 

   
  
  
  
  
  
  // update all springs rest-lengths, based on current particle position
  // the effect is, that the body keeps the current shape
  public void applySpringMemoryEffect(){
    ArrayList<DwSpringConstraint> springs = physics.getSprings();
    for(DwSpringConstraint spring : springs){
      spring.updateRestlength();
    }
  }
  
  
  
  
  
  boolean APPLY_WIND     = false;
  boolean MOVE_CAM       = false;
  boolean MOVE_PARTICLE  = false;
  boolean SNAP_PARTICLE  = false;
  float   SNAP_RADIUS    = 30;
  boolean DELETE_SPRINGS = false;
  float   DELETE_RADIUS  = 15;

  public void mousePressed(){
    if((mouseButton == LEFT || mouseButton == CENTER) && !MOVE_CAM){
      MOVE_PARTICLE = true;
    }
    if(mouseButton == RIGHT && !MOVE_CAM){
      DELETE_SPRINGS = true;
    }
  }
  
  public void mouseReleased(){
    if(!MOVE_CAM){
//      if(!DELETE_SPRINGS && particle_nearest != null){
        if(MOVE_PARTICLE && pnearest != null){
          if(mouseButton == LEFT  ) pnearest.particle.enable(true, true, true);
          if(mouseButton == CENTER) pnearest.particle.enable(true, false, false);
          pnearest = null;
        }

//      }
      if(mouseButton == RIGHT) DELETE_SPRINGS = false;
    }
   
    MOVE_PARTICLE  = false;
    DELETE_SPRINGS = false;
  }
  
  public void keyPressed(){
    if(key == CODED){
      if(keyCode == ALT){
        MOVE_CAM = true;
      }
    }
  }
  
  public void keyReleased(){
    
   
    if(key == 'r') createBodies();
    
    
    MOVE_CAM = false; 
  }

  
  public void createCam(){
    // camera - modelview
    double   distance = 1518.898;
    double[] look_at  = { 58.444, -48.939, 167.661};
    double[] rotation = { -0.744,   0.768,  -0.587};
    peasycam = new PeasyCam(this, look_at[0], look_at[1], look_at[2], distance);
    peasycam.setMaximumDistance(10000);
    peasycam.setMinimumDistance(0.1f);
    peasycam.setRotations(rotation[0], rotation[1], rotation[2]);
    cam_state_0 = peasycam.getState();
    
    // camera - projection
    float fovy    = PI/3.0f;
    float aspect  = width/(float)(height);
    float cameraZ = (height*0.5f) / tan(fovy*0.5f);
    float zNear   = cameraZ/100.0f;
    float zFar    = cameraZ*20.0f;
    perspective(fovy, aspect, zNear, zFar);
  }
  
  
  public void resetCam(){
    peasycam.setState(cam_state_0, 700);
  }
  
  public void printCam(){
    float[] pos = peasycam.getPosition();
    float[] rot = peasycam.getRotations();
    float[] lat = peasycam.getLookAt();
    float   dis = (float) peasycam.getDistance();
    
    System.out.printf(Locale.ENGLISH, "position: (%7.3f, %7.3f, %7.3f)\n", pos[0], pos[1], pos[2]);
    System.out.printf(Locale.ENGLISH, "rotation: (%7.3f, %7.3f, %7.3f)\n", rot[0], rot[1], rot[2]);
    System.out.printf(Locale.ENGLISH, "look-at:  (%7.3f, %7.3f, %7.3f)\n", lat[0], lat[1], lat[2]);
    System.out.printf(Locale.ENGLISH, "distance: (%7.3f)\n", dis);
  }
  

  
  
