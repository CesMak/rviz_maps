/*
 * AerialMapDisplay.h
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *	Created on: 07/09/2014
 */

#ifndef AERIAL_MAP_DISPLAY_H
#define AERIAL_MAP_DISPLAY_H

// NOTE: workaround for issue: https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/time.h>
#include <rviz/display.h>
#include <sensor_msgs/NavSatFix.h>
#include <rviz_maps/rviz_scale.h> // scaling message

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#endif  //  Q_MOC_RUN

#include <QObject>
#include <QtConcurrentRun>
#include <QFuture>
#include <QByteArray>
#include <QFile>
#include <QNetworkRequest>

#include <memory>
#include <tileloader.h>

namespace Ogre {
class ManualObject;
}

namespace rviz {

class FloatProperty;
class IntProperty;
class Property;
class RosTopicProperty;
class StringProperty;
class TfFrameProperty;
class EnumProperty;

/**
 * @class AerialMapDisplay
 * @brief Displays a satellite map along the XY plane.
 */
class AerialMapDisplay : public Display {
  Q_OBJECT
public:
  AerialMapDisplay();
  ~AerialMapDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void fixedFrameChanged() override;
  void reset() override;
  void update(float, float) override;

protected Q_SLOTS:
  void updateDynamicReload();
  void updateAlpha();
  void updateTopic();
  void updateFrame();
  void updateDrawUnder();
  void updateObjectURI();
  void updateZoom();
  void updateBlocks();
  void updateFrameConvention();

  //Additions:
  void updateObjectURI_MINE();
  void setOrigin();
  void plotPoints1();
  void updateScale1();

  //  slots for TileLoader messages
  void initiatedRequest(QNetworkRequest request);
  void receivedImage(QNetworkRequest request);
  void finishedLoading();
  void errorOcurred(QString description);

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  virtual void subscribe();
  virtual void unsubscribe();

  void navFixCallback(const sensor_msgs::NavSatFixConstPtr &msg);

  void loadImagery();

  void assembleScene();

  void clear();
  
  void clearGeometry();

  void transformAerialMap();

  unsigned int map_id_;
  unsigned int scene_id_;

  /// Instance of a tile w/ associated ogre data
  struct MapObject {
    Ogre::ManualObject *object;
    Ogre::TexturePtr texture;
    Ogre::MaterialPtr material;
  };
  std::vector<MapObject> objects_;

  ros::Subscriber coord_sub_;

  //  properties
  RosTopicProperty *topic_property_;
  TfFrameProperty *frame_property_;
  Property *dynamic_reload_property_;
  StringProperty *object_uri_property_;
  IntProperty *zoom_property_;
  IntProperty *blocks_property_;
  FloatProperty *resolution_property_;
  FloatProperty *alpha_property_;
  Property *draw_under_property_;
  EnumProperty * frame_convention_property_;

  float alpha_;
  bool draw_under_;
  std::string object_uri_;
  int zoom_;
  int blocks_;

  //  tile management
  bool dirty_;
  bool received_msg_;
  sensor_msgs::NavSatFix ref_fix_; //coordinate center.

  std::shared_ptr<TileLoader> loader_;

  //Additions:
  EnumProperty *object_uri_property_mine_;
  StringProperty *longitude_of_topic1_property_mine_;
  StringProperty *latitude_of_topic1_property_mine_;
  RosTopicProperty *topic_property_plot_points1;
  FloatProperty *scale_point1_property_;
  sensor_msgs::NavSatFix point1_;
  void navFixCallback_points1(const sensor_msgs::NavSatFixConstPtr &msg);
  float scale_point1_; // radius of points in [m]
  ros::Publisher  points1_publish_;  // my publisher
  virtual void initialize_publish();
};

} // namespace rviz

#endif
