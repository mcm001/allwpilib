/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.cameraserver;

/// A sink that acts as a MJPEG-over-HTTP network server.
public class HTTPSink extends VideoSink {
  /// Create a MJPEG-over-HTTP server sink.
  /// @param name Sink name (arbitrary unique identifier)
  /// @param listenAddress TCP listen address (empty string for all addresses)
  /// @param port TCP port number
  public HTTPSink(String name, String listenAddress, int port) {
    super(CameraServerJNI.createHTTPSink(name, listenAddress, port));
  }

  /// Create a MJPEG-over-HTTP server sink.
  /// @param name Sink name (arbitrary unique identifier)
  /// @param port TCP port number
  public HTTPSink(String name, int port) {
    this(name, "", port);
  }
}
