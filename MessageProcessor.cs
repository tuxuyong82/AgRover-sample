using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using CommonLib;

namespace ScoutingRobotCotrol {

	/// <summary>
	/// This class Provide the function to Send and Receive messages to/from the Robot
	/// </summary>
	public class MessageProcessorC {

		/// <summary>
		/// Udp port
		/// </summary>
		NetClient UdpPort;

		/// <summary>
		/// Reference to OrientationModule
		/// </summary>
		OrientationModule om;

		public int RX_CNT;
		public event SettingUpdateDelegate OnSettingUpdate;

		/// <summary>
		/// Send Communication parameters
		/// </summary>
		/// <param name="MyIP">Base PC's IP Address</param>
		/// <param name="MyPort">Base PC's Port No</param>
		/// <param name="DestIP">Veichle PC's IP Address</param>
		/// <param name="DestPort">Veichle PC's Port No</param>
		public void SetIPAddress( string MyIP, int MyPort, string DestIP, int DestPort ) {
			IPAddress myIP = System.Net.IPAddress.Parse( MyIP );
			IPAddress deIP = System.Net.IPAddress.Parse( DestIP );
			byte[] MyOctets;
			byte[] DeOctets;
			MyOctets = myIP.GetAddressBytes();
			DeOctets = deIP.GetAddressBytes();

			// Build a Packet.
			int n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.NetConfig;	//Type Of Data
			Tx_Buff[ n++ ] = 14;							//Packet Length
			Tx_Buff[ n++ ] = MyOctets[ 0 ];
			Tx_Buff[ n++ ] = MyOctets[ 1 ];
			Tx_Buff[ n++ ] = MyOctets[ 2 ];
			Tx_Buff[ n++ ] = MyOctets[ 3 ];
			Tx_Buff[ n++ ] = ( byte )MyPort;
			Tx_Buff[ n++ ] = ( byte )( MyPort >> 8 );
			Tx_Buff[ n++ ] = DeOctets[ 0 ];
			Tx_Buff[ n++ ] = DeOctets[ 1 ];
			Tx_Buff[ n++ ] = DeOctets[ 2 ];
			Tx_Buff[ n++ ] = DeOctets[ 3 ];
			Tx_Buff[ n++ ] = ( byte )DestPort;
			Tx_Buff[ n++ ] = ( byte )( DestPort >> 8 );
			//Set length of this messsage
			g_SCI2_TX_LEN = n;


		}

		/// <summary>
		/// Send a message to the Veichle
		/// </summary>
		public void SendPacket() {
			//If nothing to send, send the "notify" to confirm if the connection is alive or not.
			if( g_SCI2_TX_LEN == SCI2_HEAD_LEN ) {
				SetNotify();
			}
			PreparePacket();
			UdpPort.SendUDPMessage( Tx_Buff, Tx_Buff[ 2 ] );
			g_SCI2_TX_LEN = SCI2_HEAD_LEN;
		}

		/// <summary>
		/// Stop Receiving the packets
		/// </summary>
		public void StopRecceiving() {
			UdpPort.StopReceiving();
		}

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="MyUdpPort">Port number of this computer</param>
		/// <param name="DestUdpPort">Port number of the robot</param>
		/// <param name="destIP">IP Address of the robot</param>
		public MessageProcessorC( int MyUdpPort, int DestUdpPort, IPAddress destIP, OrientationModule om ) {
			this.om = om;
			UdpPort = new NetClient( MyUdpPort, DestUdpPort, destIP );
			UdpPort.AfterReceive += new NetClient.UdpReceivedEventHandler( UdpPort_AfterReceive );
			UdpPort.StartReceiving();
			Tx_Buff[ 0 ] = 0xFF;	//This value is never changed
			Tx_Buff[ 1 ] = 0x80;	//This value is never changed
			g_SCI2_TX_LEN = SCI2_HEAD_LEN;
		}

		/// <summary>
		/// This function will be called after receiving a Packet.
		/// And copy data to the buffer and Call another function to process messages in the packet.
		/// </summary>
		/// <param name="sender">--</param>
		/// <param name="e">received message</param>
		void UdpPort_AfterReceive( object sender, UdpReceivedEventArgs e ) {
			int len = e.date.Length;
			for( int i = 0 ; i < len ; i++ ) {
				//Copy
				this.Rx_Buff[ i ] = e.date[ i ];
			}
			this.ProcessPacket( len );
		}

		/// <summary>
		/// Receve buffer
		/// </summary>
		public byte[] Rx_Buff = new byte[ 300 ];
		/// <summary>
		/// Transmit buffer
		/// </summary>
		public byte[] Tx_Buff = new byte[ 300 ];

		/// <summary>
		/// This value is used to recognize the start of the packet
		/// </summary>
		const byte SOH1 = 0xFF;
		/// <summary>
		/// This value is used to recognize the start of the packet
		/// </summary>
		const byte SOH2 = 0x80;
		/// <summary>
		/// Header length is 5 bytes
		/// </summary>
		const int SCI2_HEAD_LEN = 5;
		/// <summary>
		/// This value will be used to recognize the end of the packet
		/// </summary>
		const int C_CR = 0x0d;	// CR

		/// <summary>
		/// Retrieve message in the Packet
		/// </summary>
		/// <param name="len"></param>
		private void ProcessPacket( int len ) {
			RX_CNT++;

			if( len > 200 )
				return;
			bool b_SOH1 = false;
			//			bool b_SOH2 = false;
			int Data_Begin_Pos = 0;
			int data_L = 0;

			for( int i = 0 ; i < len ; i++ ) {
				if( b_SOH1 ) {
					if( Rx_Buff[ i ] == SOH2 ) {
						if( i > Rx_Buff.Length - 5 )
							return;
						data_L = Rx_Buff[ i + 1 ];
						if( len >= data_L + Data_Begin_Pos ) {
							if( Rx_Buff[ i + data_L - 2 ] == 0x0D ) {
								if( true ) {
									i = Data_Begin_Pos + data_L - 1;
									int pos = Data_Begin_Pos + 5;
									for( int j = pos ; j < Data_Begin_Pos + data_L - 2 ; ) {
										switch( Rx_Buff[ j ] ) {
											case ( int )MessageType.AnalogBoardBaseAddressChanged:
												GetAnalogBoardBaseAddress( Rx_Buff, j );
												j += Rx_Buff[ j + 1 ];
												break;
											case ( int )MessageType.EncorderBoardBaseAddressChanged:
												GetEncoderBoardBaseAddress( Rx_Buff, j );
												j += Rx_Buff[ j + 1 ];
												break;
											case ( int )MessageType.SteerAngles:
												GetSteeringAngles( Rx_Buff, j );
												j += Rx_Buff[ j + 1 ];
												break;
											default:
												return;
										}
									}
									b_SOH1 = false;
								}
							}
						}
						else {
							b_SOH1 = false;
						}
					}
					else {
						b_SOH1 = false;
					}
				}
				else {
					if( Rx_Buff[ i ] == SOH1 ) {
						b_SOH1 = true;
						Data_Begin_Pos = i;
					}
				}
			}
		}

		int g_SCI2_TX_LEN = SCI2_HEAD_LEN;
		int g_SCI2_TX_NUM = 0;

		/// <summary>
		/// Set "Height Packet"
		/// </summary>
		/// <param name="height">height of the robot (0 - 255)</param>
		public void SetHeight( int height ) {
			int n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.Height;
			Tx_Buff[ n++ ] = 6;
			Tx_Buff[ n++ ] = ( byte )height; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			g_SCI2_TX_LEN = n;

		}


		/// <summary>
		/// Set "Notify Packet"
		/// </summary>
		/// <param name="height">Height of the robot (0-255)</param>
		public void SetNotify() {
			int n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.Notify;
			Tx_Buff[ n++ ] = 6;	//Packet Length
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			g_SCI2_TX_LEN = n;
		}

		/// <summary>
		/// Set "Reset Motor Packet"
		/// </summary>
		public void SetResetMotor() {
			int n;

			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.RestartRequest;
			Tx_Buff[ n++ ] = 6;	//Packet Length
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			g_SCI2_TX_LEN = n;

		}


		/// <summary>
		/// Set "Emergensy Packet"
		/// </summary>
		public void SetEmergencyBrake() {
			int n;
			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.EmergencyBrakeChanged;
			Tx_Buff[ n++ ] = 6;	//Packet Length
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			g_SCI2_TX_LEN = n;

		}

		/// <summary>
		/// Set "Bake Packet"
		/// </summary>
		/// <param name="brakeforce">brakeforce of wheel (0-255)</param>
		public void SetBrake( int brakeforce ) {
			int n;
			byte[] buff;
			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.BrakeChanged;  	//Type Of Data
			Tx_Buff[ n++ ] = 6;	//Packet Length

			buff = BitConverter.GetBytes( brakeforce );

			Tx_Buff[ n++ ] = buff[ 0 ]; //dummy
			Tx_Buff[ n++ ] = buff[ 1 ]; //dummy
			Tx_Buff[ n++ ] = buff[ 2 ]; //dummy
			Tx_Buff[ n++ ] = buff[ 3 ]; //dummy
			g_SCI2_TX_LEN = n;

		}

		/// <summary>
		/// Set "Key Switch(steering motor) Packet"
		/// </summary>
		/// <param name="height">true is on , false is off</param>
		public void SetKeySwitch( bool onoff ) {
			int n;

			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.KeySwitchChanged;  	//Type Of Data
			Tx_Buff[ n++ ] = 6;	//Packet Length

			if( onoff ) {
				Tx_Buff[ n++ ] = 1; //dummy
			}
			else {
				Tx_Buff[ n++ ] = 0; //dummy
			}
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			g_SCI2_TX_LEN = n;

		}

		/// <summary>
		/// Set "Steering Mode Packet"
		/// </summary>
		public void SetSteeringMode( SteeringMode sm ) {
			int n;
			byte[] buff;
			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.SteeringModeChanged;  	//Type Of Data
			Tx_Buff[ n++ ] = 6;	//Packet Length

			buff = BitConverter.GetBytes( ( int )sm );

			Tx_Buff[ n++ ] = buff[ 0 ]; //dummy
			Tx_Buff[ n++ ] = buff[ 1 ]; //dummy
			Tx_Buff[ n++ ] = buff[ 2 ]; //dummy
			Tx_Buff[ n++ ] = buff[ 3 ]; //dummy
			g_SCI2_TX_LEN = n;

		}

		/// <summary>
		/// Set "Setting Mode Off Packet"
		/// </summary>
		public void SetSettingModeOff() {
			int n;
			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.SettingModeOff;  	//Type Of Data
			Tx_Buff[ n++ ] = 6;	//Packet Length
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			g_SCI2_TX_LEN = n;

		}

		/// <summary>
		/// Set "Setting Mode On Packet"
		/// </summary>
		public void SetSettingModeOn() {
			int n;
			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.SettingModeOn;  	//Type Of Data
			Tx_Buff[ n++ ] = 6;	//Packet Length
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			g_SCI2_TX_LEN = n;

		}

		/// <summary>
		/// Get Encoder Board BaseAddress from Veicle PC
		/// </summary>
		/// <param name="data">data</param>
		/// <param name="pos">position</param>
		public void GetEncoderBoardBaseAddress( byte[] data, int pos ) {
			if( OnSettingUpdate != null ) {
				int BaseAddress = BitConverter.ToInt32( data, pos + 2 );
				OnSettingUpdate( this, new SeetingUpdateEventArgs( SettingEvents.EncorderBoardBaseAddressChanged, WheelPos.ALL, BaseAddress, 0 ) );
			}
		}

		/// <summary>
		/// Make "EncoderBoard Baseaddress Packet" ready to send
		/// </summary>
		public void SetEncoderBoardBaseAddress( int BaseAddress ) {
			int n;
			byte[] buff;

			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.EncorderBoardBaseAddressChanged;  	//Type Of Data
			Tx_Buff[ n++ ] = 6;	//Packet Length

			buff = BitConverter.GetBytes( BaseAddress );
			Tx_Buff[ n++ ] = buff[ 0 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 1 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 2 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 3 ]; //Set Angle
			g_SCI2_TX_LEN = n;
		}

		/// <summary>
		/// Get Analog Board BaseAddress from Veicle PC
		/// </summary>
		/// <param name="data">data</param>
		/// <param name="pos">position</param>
		public void GetAnalogBoardBaseAddress( byte[] data, int pos ) {
			if( OnSettingUpdate != null ) {
				int BaseAddress = BitConverter.ToInt32( data, pos + 2 );
				OnSettingUpdate( this, new SeetingUpdateEventArgs( SettingEvents.AnalogBoardBaseAddressChanged, WheelPos.ALL, BaseAddress, 0 ) );
			}
		}

		/// <summary>
		/// Make "AnalogBoard BaseAddress Packe" ready to send
		/// </summary>
		public void SetAnalogBoardBaseAddress( int BaseAddress ) {
			int n;
			byte[] buff;

			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.AnalogBoardBaseAddressChanged;  	//Type Of Data
			Tx_Buff[ n++ ] = 6;	//Packet Length

			buff = BitConverter.GetBytes( BaseAddress );
			Tx_Buff[ n++ ] = buff[ 0 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 1 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 2 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 3 ]; //Set Angle
			g_SCI2_TX_LEN = n;
		}

		/// <summary>
		/// Make "Max Speedd Packe" ready to send
		/// </summary>
		public void SetMaxSpeed( double Value ) {
			int n;
			byte[] buff;

			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.MaxSpeedChanged;  	//Type Of Data
			Tx_Buff[ n++ ] = 10;	//Packet Length

			buff = BitConverter.GetBytes( Value );
			Tx_Buff[ n++ ] = buff[ 0 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 1 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 2 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 3 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 4 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 5 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 6 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 7 ]; //Set Angle
			g_SCI2_TX_LEN = n;

		}

		/// <summary>
		/// Make "PIS Parameters Packe" ready to send
		/// </summary>
		public void SetPIDParam( double Value, string param ) {
			int n, p = 0;
			byte[] buff;

			switch( param ) {
				case "SKP":
					p = ( int )MessageType.SteerKP_Changed;
					break;
				case "SKI":
					p = ( int )MessageType.SteerKI_Changed;
					break;
				case "SKD":
					p = ( int )MessageType.SteerKD_Changed;
					break;
				case "DKP":
					p = ( int )MessageType.DriveKP_Changed;
					break;
				case "DKI":
					p = ( int )MessageType.DriveKI_Changed;
					break;
				case "DKD":
					p = ( int )MessageType.DriveKD_Changed;
					break;
			}

			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( byte )p;  	//Type Of Data
			Tx_Buff[ n++ ] = 10;		//Packet Length

			buff = BitConverter.GetBytes( Value );
			Tx_Buff[ n++ ] = buff[ 0 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 1 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 2 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 3 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 4 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 5 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 6 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 7 ]; //Set Angle
			g_SCI2_TX_LEN = n;

		}



		/// <summary>
		/// Make "Speed and Direction Packet" ready to send
		/// </summary>
		public void SetSpeedAndDirection( int speed, float dir ) {
			int n;
			byte[] buff;

			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.SpeedDirectionBrakeChanged;  	//Type Of Data
			Tx_Buff[ n++ ] = 10;	//Packet Length
			buff = BitConverter.GetBytes( speed );
			Tx_Buff[ n++ ] = buff[ 0 ]; //Set Speed
			Tx_Buff[ n++ ] = buff[ 1 ]; //Set Speed
			Tx_Buff[ n++ ] = buff[ 2 ]; //Set Speed
			Tx_Buff[ n++ ] = buff[ 3 ]; //Set Speed

			buff = BitConverter.GetBytes( dir );
			Tx_Buff[ n++ ] = buff[ 0 ]; //Set Direction
			Tx_Buff[ n++ ] = buff[ 1 ]; //Set Direction
			Tx_Buff[ n++ ] = buff[ 2 ]; //Set Direction
			Tx_Buff[ n++ ] = buff[ 3 ]; //Set Direction
			g_SCI2_TX_LEN = n;
		}

		public void GetSteeringAngles( byte[] data, int pos ) {
			om.FL_mAngle = data[ pos + 2 ] + data[ pos + 3 ] * 255 - 500;
			om.FR_mAngle = data[ pos + 4 ] + data[ pos + 5 ] * 255 - 500;
			om.RL_mAngle = data[ pos + 6 ] + data[ pos + 7 ] * 255 - 500;
			om.RR_mAngle = data[ pos + 8 ] + data[ pos + 9 ] * 255 - 500;
		}

		/// <summary>
		/// Make "Steering Angle Packe(for setting mode)" ready to send
		/// </summary>
		public void SetSteeringAngle( WheelPos ws, double angle ) {
			int n;

			byte[] buff;

			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.SteerAngleChanged;  	//Type Of Data
			Tx_Buff[ n++ ] = 14;	//Packet Length

			buff = BitConverter.GetBytes( ( int )ws );
			Tx_Buff[ n++ ] = buff[ 0 ]; //Set Wheel Pos
			Tx_Buff[ n++ ] = buff[ 1 ]; //Set Wheel Pos
			Tx_Buff[ n++ ] = buff[ 2 ]; //Set Wheel Pos
			Tx_Buff[ n++ ] = buff[ 3 ]; //Set Wheel Pos

			buff = BitConverter.GetBytes( angle );
			Tx_Buff[ n++ ] = buff[ 0 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 1 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 2 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 3 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 4 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 5 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 6 ]; //Set Angle
			Tx_Buff[ n++ ] = buff[ 7 ]; //Set Angle
			g_SCI2_TX_LEN = n;

		}

		/// <summary>
		/// Make "Origin Angle Apply Packet(for setting mode)" ready to send
		/// </summary>
		public void SetOrigiAngleApply() {
			int n;
			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.OriginAngleApplied;  	//Type Of Data
			Tx_Buff[ n++ ] = 6;	//Packet Length
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			g_SCI2_TX_LEN = n;

		}

		/// <summary>
		/// Make "Rest request Packet" ready to send
		/// </summary>
		public void SetRestartRequest() {
			int n;
			n = g_SCI2_TX_LEN;
			Tx_Buff[ n++ ] = ( int )MessageType.RestartRequest;  	//Type Of Data
			Tx_Buff[ n++ ] = 6;	//Packet Length
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			Tx_Buff[ n++ ] = 0; //dummy
			g_SCI2_TX_LEN = n;
		}

		/// <summary>
		/// Make all packets ready to be sent
		/// </summary>
		public void PreparePacket() {
			Tx_Buff[ g_SCI2_TX_LEN++ ] = 0xCC;
			Tx_Buff[ g_SCI2_TX_LEN++ ] = C_CR;
			Tx_Buff[ 2 ] = ( byte )g_SCI2_TX_LEN;

			if( g_SCI2_TX_NUM == 0xFFFF )
				g_SCI2_TX_NUM = 0;
			Tx_Buff[ 3 ] = ( byte )g_SCI2_TX_NUM;
			Tx_Buff[ 4 ] = ( byte )( g_SCI2_TX_NUM >> 8 );
			g_SCI2_TX_NUM++;
		}
	}
}
