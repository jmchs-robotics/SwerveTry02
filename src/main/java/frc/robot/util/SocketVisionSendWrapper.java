package frc.robot.util;

import frc.robot.util.SocketVisionSender;

/**
 * Wrapper for SocketVisionSender (one-to-one) so the threaded objects can be handled in an FRC-safe
 * manner and be passed around commands in a more generic way.
 */
public class SocketVisionSendWrapper {

  private SocketVisionSender m_sender = null;

  private int m_port;
  private String m_ip;

  public SocketVisionSendWrapper( String ip, int port){
    m_ip = ip;
    m_port = port;
  }

  public void init(){
    if( m_sender == null) {
			m_sender = new SocketVisionSender(m_ip, m_port);

			m_sender.start();
    }
  }

  public void shutDown(){
    if(m_sender != null) {
			try {
				m_sender.stoprunning();
				m_sender.join();
				m_sender = null;
			}catch (Exception e) {
				e.printStackTrace();
			}
		}
  }

  public SocketVisionSender get(){
    return m_sender;
  }
}