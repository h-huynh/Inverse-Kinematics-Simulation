
void setup(){
  size(1200,800);
  surface.setTitle("Project 3: Inverse Kinematics");
}

//Skeleton structure//

//Rotational limits: when true, rotations of the joints are constrained.
boolean r_limit = true;
float t = 0.05;

//Root ------
Vec2 root = new Vec2(600,400);
Vec2 offset = new Vec2(100,0);
Vec2 l_root = root.minus(offset);
Vec2 r_root = root.plus(offset);

//Left arm ------

//Link lengths one through four
float[] l = {60,60,60,40};

//Angle lengths one through four
float[] a = {0.3,0.3,0.3,0.3};

//Right arm ------

//Link lengths one through four
float[] r = {60,60,60,40};

//Angle lengths one through four
float[] b = {0.3,0.3,0.3,0.3};

//FK variables
Vec2[] left_arm;
Vec2[] right_arm;

void solve(){
  Vec2 goal = new Vec2(mouseX,mouseY);
  
  Vec2 startToGoal,startToGoal2, startToEndEffector,startToEndEffector2;
  float dotProd, dotProd2, angleDiff,angleDiff2;
  
  //Solve both left_arm and right_arm
  for (int i = 2; i >= 0; i--){
    startToGoal = goal.minus(left_arm[i]);
    startToGoal2 = goal.minus(right_arm[i]);
    
    startToEndEffector = left_arm[3].minus(left_arm[i]);
    startToEndEffector2 = right_arm[3].minus(right_arm[i]);
    
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    
    dotProd2 = dot(startToGoal2.normalized(),startToEndEffector2.normalized());
    dotProd2 = clamp(dotProd2,-1,1);
    
    angleDiff = acos(dotProd);
    if (cross(startToGoal,startToEndEffector) < 0)
      a[i+1] = lerp(a[i+1],a[i+1]+angleDiff,t);
    else
      a[i+1] = lerp(a[i+1],a[i+1]-angleDiff,t);
      
    angleDiff2 = acos(dotProd2);
    if (cross(startToGoal2,startToEndEffector2) < 0)
      b[i+1] = lerp(b[i+1],b[i+1]+angleDiff2,t);
    else
      b[i+1] = lerp(b[i+1],b[i+1]-angleDiff2,t);
    
    checkAngleLimits(i+1);
    fk();
  }
  
  //Update shoulder joints
  
  //Left
  startToGoal = goal.minus(l_root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = left_arm[3].minus(l_root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a[0] = lerp(a[0],a[0]+angleDiff,t);
  else
    a[0] = lerp(a[0],a[0]-angleDiff,t);
  
  //Right
  startToGoal2 = goal.minus(r_root);
  if (startToGoal2.length() < .0001) return;
  startToEndEffector2 = right_arm[3].minus(r_root);
  dotProd2 = dot(startToGoal2.normalized(),startToEndEffector2.normalized());
  dotProd2 = clamp(dotProd2,-1,1);
  angleDiff2 = acos(dotProd2);
  if (cross(startToGoal2,startToEndEffector2) < 0)
    b[0] = lerp(b[0],b[0]+angleDiff2,t);
  else
    b[0] = lerp(b[0],b[0]-angleDiff2,t);  
  checkAngleLimits(0);
  fk();  
}



void checkAngleLimits(int i){

  //Checks if rotational limits are active
  if (r_limit == false)
    return;
  
  //Limits
  switch(i){
    case 0:
      if(b[i] < 0)
        b[i] = 0;
      if (b[i] > HALF_PI)
        b[i] = HALF_PI;
      if(a[i] < HALF_PI)
        a[i] = HALF_PI;
      if (a[i] > PI)
        a[i] = PI;
      break;
    case 1:
    case 2:
    case 3:
      if(b[i] < -HALF_PI)
        b[i] = -HALF_PI;
      if (b[i] > HALF_PI)
        b[i] = HALF_PI;
      if(a[i] < -HALF_PI)
        a[i] = -HALF_PI;
      if (a[i] > HALF_PI)
        a[i] = HALF_PI;
      break;
  }
  
}
void fk(){
  left_arm = new Vec2[4];
  right_arm = new Vec2[4];
  
  left_arm[0] = new Vec2(cos(a[0])*l[0],sin(a[0])*l[0]).plus(l_root);
  left_arm[1] = new Vec2(cos(a[0]+a[1])*l[1],sin(a[0]+a[1])*l[1]).plus(left_arm[0]);
  left_arm[2] = new Vec2(cos(a[0]+a[1]+a[2])*l[2],sin(a[0]+a[1]+a[2])*l[2]).plus(left_arm[1]);
  left_arm[3] = new Vec2(cos(a[0]+a[1]+a[2]+a[3])*l[3],sin(a[0]+a[1]+a[2]+a[3])*l[3]).plus(left_arm[2]);
  
  right_arm[0] = new Vec2(cos(b[0])*r[0],sin(b[0])*r[0]).plus(r_root);
  right_arm[1] = new Vec2(cos(b[0]+b[1])*r[1],sin(b[0]+b[1])*r[1]).plus(right_arm[0]);
  right_arm[2] = new Vec2(cos(b[0]+b[1]+b[2])*r[2],sin(b[0]+b[1]+b[2])*r[2]).plus(right_arm[1]);
  right_arm[3] = new Vec2(cos(b[0]+b[1]+b[2]+b[3])*r[3],sin(b[0]+b[1]+b[2]+b[3])*r[3]).plus(right_arm[2]);
}

//Arm width
float armW = 40;

void draw(){
  fk();
  solve();
  
  background(250,250,250);
  fill(160,160,160);
  circle(600,400,200);
 
  //Left arm
  pushMatrix();
  translate(l_root.x,l_root.y);
  rotate(a[0]);
  circle(0,0,armW);
  rect(0, -armW/2, l[0], armW);
  popMatrix();
  
  pushMatrix();
  translate(left_arm[0].x,left_arm[0].y);
  rotate(a[0]+a[1]);
  circle(0,0,armW);
  rect(0, -armW/2, l[1], armW);
  popMatrix();
  
  pushMatrix();
  translate(left_arm[1].x,left_arm[1].y);
  rotate(a[0]+a[1]+a[2]);
  circle(0,0,armW);
  rect(0, -armW/2, l[2], armW);
  popMatrix();
  
  pushMatrix();
  translate(left_arm[2].x,left_arm[2].y);
  rotate(a[0]+a[1]+a[2]+a[3]);
  circle(0,0,armW);
  rect(0, -armW/2, l[3], armW);
  popMatrix();
  
  //Right arm
  pushMatrix();
  translate(r_root.x,r_root.y);
  rotate(b[0]);
  circle(0,0,armW);
  rect(0, -armW/2, r[0], armW);
  popMatrix();
  
  pushMatrix();
  translate(right_arm[0].x,right_arm[0].y);
  rotate(b[0]+b[1]);
  circle(0,0,armW);
  rect(0, -armW/2, r[1], armW);
  popMatrix();
  
  pushMatrix();
  translate(right_arm[1].x,right_arm[1].y);
  rotate(b[0]+b[1]+b[2]);
  circle(0,0,armW);
  rect(0, -armW/2, r[2], armW);
  popMatrix();
  
  pushMatrix();
  translate(right_arm[2].x,right_arm[2].y);
  rotate(b[0]+b[1]+b[2]+b[3]);
  circle(0,0,armW);
  rect(0, -armW/2, r[3], armW);
  popMatrix();
  
  fill(0,0,0);
  pushMatrix();
  translate(mouseX,mouseY);
  circle(0,0,40);
  popMatrix();
  
}
   
//-----------------
// Vector Library
//-----------------

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
