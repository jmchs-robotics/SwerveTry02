package frc.robot.util;

import frc.robot.util.SocketVision;

/**
 * Wrapper for SocketVisionSender (one-to-one) so the threaded objects can be handled in an FRC-safe
 * manner and be passed around commands in a more generic way.
 */
public class SocketVisionWrapper {

  private SocketVision m_reader = null;

  private int m_port;
  private String m_ip;

  public SocketVisionWrapper( String ip, int port){
    m_ip = ip;
    m_port = port;
  }

  public void init(){
    if( m_reader == null) {
			m_reader = new SocketVision(m_ip, m_port);

			m_reader.start();
    }
  }

  public void shutDown(){
    if(m_reader != null) {
			try {
				m_reader.stoprunning();
				m_reader.join();
				m_reader = null;
			}catch (Exception e) {
				e.printStackTrace();
			}
		}
  }

  public SocketVision get(){
    return m_reader;
  }
}