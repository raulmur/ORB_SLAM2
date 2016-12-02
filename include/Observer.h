#include <iostream>
#include <vector>

using namespace std;


#ifndef __OBSERVER_H__
#define __OBSERVER_H__
namespace ORB_SLAM2{
class Observer ;
class Subject ;

class Observer {
    Subject * model;
  public:
    Observer();
    Observer(Subject *mod);
    void setSubject(Subject * mod);
    virtual void update() = 0;
  protected:
    Subject *getSubject() {
        return model;
    }
};

class Subject {
    vector < Observer * > views;
  public:
    void attach(Observer *obs);
    void update();
};

}

#endif
