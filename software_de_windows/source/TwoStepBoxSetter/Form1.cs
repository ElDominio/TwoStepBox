using System;
using System.ComponentModel;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Windows.Forms;

namespace TwoStepBoxSetter;

public class Form1 : Form
{
	private string recievedData = "";

	private bool colorDirection;

	private byte faderTime;

	private IContainer components;

	private ComboBox COMbox;

	private Label label1;

	private Button connectBtn;

	private TextBox rpmBox;

	private Label label2;

	private Label label3;

	private TextBox hystBox;

	private Label label4;

	private TextBox posBox;

	private Button sendButton;

	private Timer timer1;

	private GroupBox groupBox1;

	private TextBox testBox;

	private Label xlabel;

	private Timer timer2;

	private SerialPort serialPort1;

	private Label rpmRead;

	private Label rpmLabel;

	private Label potPosLabel;

	private Label potPosVal;

	private Label label6;

	private Label rpmHystVal;

	private CheckBox toyotaCheckbox;

	private Label cylCount;

	private Label cylLabel;

	private GroupBox coilSetBox;

	private GroupBox extraBox;

	private Label coilVal;

	private Label label7;

	private Label RPMfilterVal;

	private Label label9;

	private Label toyotaVal;

	private Label label8;

	private Label armedVal;

	private Label label11;

	private GroupBox toyotaBox;

	private RadioButton eightCylToyo;

	private RadioButton sixCylToyo;

	private RadioButton fourCylToyo;

	private Label cutRPMval;

	private Label label10;

	private GroupBox States;

	private Timer timer3;

	private Button rpmButton;

	private Button clutchButton;

	private ComboBox coilBox;

	private CheckBox checkBox1;

	private Label label12;

	private Label label5;

	private ComboBox triggerBox;

	private Label trigVal;

	private Label label14;

	public Form1()
	{
		InitializeComponent();
	}

	private void Form1_Load(object sender, EventArgs e)
	{
		updatePorts();
		serialPort1.DataReceived += serialPort1_DataReceived;
		xlabel.ForeColor = Color.FromArgb(0, 0, 0);
		rpmHystVal.Text = "";
		toyotaVal.Text = "";
		RPMfilterVal.Text = "";
		potPosVal.Text = "";
		coilVal.Text = "";
		armedVal.Text = "";
		cutRPMval.Text = "";
		rpmRead.Text = "";
		cutRPMval.Text = "";
		cylCount.Text = "";
		trigVal.Text = "";
	}

	private void updatePorts()
	{
		string[] portNames = SerialPort.GetPortNames();
		string[] array = portNames;
		foreach (string text in array)
		{
			if (COMbox.FindString(text) < 0)
			{
				COMbox.Items.Add(text);
			}
		}
		if (portNames.Length < 1)
		{
			COMbox.Items.Clear();
			disconnect();
		}
	}

	private void label1_Click(object sender, EventArgs e)
	{
	}

	private void button3_Click(object sender, EventArgs e)
	{
		if (rpmBox.Text != "" && hystBox.Text != "" && posBox.Text != "")
		{
			sendData();
			new Form2().ShowDialog();
		}
	}

	private void sendData()
	{
		serialPort1.Write("p");
		serialPort1.Write(posBox.Text);
		serialPort1.Write(rpmBox.Text);
		serialPort1.Write(hystBox.Text);
		if (hystBox.TextLength < 4)
		{
			serialPort1.Write(";");
		}
		posBox.Clear();
		rpmBox.Clear();
		hystBox.Clear();
	}

	private void positionBox_TextChanged(object sender, EventArgs e)
	{
	}

	private void hystBox_TextChanged(object sender, EventArgs e)
	{
	}

	private void COMlist_SelectedIndexChanged(object sender, EventArgs e)
	{
	}

	private void connectCom_Click(object sender, EventArgs e)
	{
		if (serialPort1.IsOpen)
		{
			disconnect();
		}
		else
		{
			connect();
		}
	}

	private void connect()
	{
		bool flag = false;
		if (COMbox.SelectedIndex != -1)
		{
			serialPort1.PortName = COMbox.Text;
			serialPort1.BaudRate = 115200;
			serialPort1.Parity = Parity.None;
			serialPort1.DataBits = 8;
			try
			{
				serialPort1.Open();
			}
			catch (UnauthorizedAccessException)
			{
				flag = true;
			}
			catch (IOException)
			{
				flag = true;
			}
			catch (ArgumentException)
			{
				flag = true;
			}
			if (flag)
			{
				MessageBox.Show(this, "Could not open the COM port. Most likely it is already in use, has been removed, or is unavailable.", "COM Port unavailable", MessageBoxButtons.OK, MessageBoxIcon.Hand);
			}
		}
		else
		{
			MessageBox.Show("Please select all the COM Serial Port Settings", "Serial Port Interface", MessageBoxButtons.OK, MessageBoxIcon.Hand);
		}
		if (serialPort1.IsOpen)
		{
			connectBtn.Text = "Disconnect";
			sendButton.Enabled = true;
			groupBox1.Enabled = true;
			coilSetBox.Enabled = true;
			extraBox.Enabled = true;
			coilBox.Enabled = true;
			xlabel.ForeColor = Color.FromArgb(255, 255, 255);
			timer1.Start();
			timer3.Start();
		}
	}

	private void disconnect()
	{
		if (serialPort1.IsOpen)
		{
			serialPort1.Write("r");
		}
		serialPort1.Close();
		connectBtn.Text = "Connect";
		xlabel.ForeColor = Color.FromArgb(0, 0, 0);
		timer1.Stop();
		sendButton.Enabled = false;
		groupBox1.Enabled = false;
		coilSetBox.Enabled = false;
		extraBox.Enabled = false;
		coilBox.Enabled = false;
		rpmHystVal.Text = "";
		toyotaVal.Text = "";
		RPMfilterVal.Text = "";
		potPosVal.Text = "";
		coilVal.Text = "";
		armedVal.Text = "";
		cutRPMval.Text = "";
		rpmRead.Text = "";
		cutRPMval.Text = "";
		cylCount.Text = "";
		trigVal.Text = "";
	}

	private void Form1_FormClosing(object sender, FormClosingEventArgs e)
	{
		if (serialPort1.IsOpen)
		{
			serialPort1.Close();
		}
	}

	private void timer1_Tick(object sender, EventArgs e)
	{
		if (recievedData.Length > 0)
		{
			if (recievedData == "ok")
			{
				timer3.Stop();
			}
			else if (recievedData[0] == 'r')
			{
				recievedData = recievedData.Replace("r", string.Empty);
				rpmRead.Text = recievedData;
			}
			else if (recievedData[0] == 'v')
			{
				potPosVal.Text = recievedData[1].ToString();
				rpmHystVal.Text = recievedData[2].ToString() + recievedData[3] + recievedData[4];
				if (recievedData[5] == '0')
				{
					armedVal.Text = "Off";
				}
				else if (recievedData[5] == '1')
				{
					armedVal.Text = "On";
				}
				cylCount.Text = recievedData[6].ToString();
				if (recievedData[7] == '0')
				{
					coilVal.Text = "Off";
				}
				else if (recievedData[7] == '1')
				{
					coilVal.Text = "On";
				}
				if (recievedData[8] == '0')
				{
					toyotaVal.Text = "Off";
				}
				else if (recievedData[8] == '1')
				{
					toyotaVal.Text = "On";
				}
				if (recievedData[9] == '0')
				{
					RPMfilterVal.Text = "Off";
				}
				else if (recievedData[9] == '1')
				{
					RPMfilterVal.Text = "On";
				}
				cutRPMval.Text = recievedData[10].ToString() + recievedData[11] + recievedData[12] + recievedData[13];
				trigVal.Text = recievedData[14].ToString();
			}
			recievedData = string.Empty;
		}
		if (faderTime > 60)
		{
			if (!colorDirection)
			{
				int b = xlabel.ForeColor.B;
				int r = xlabel.ForeColor.R;
				int g = xlabel.ForeColor.G;
				b -= 5;
				g -= 5;
				r -= 5;
				if (b < 0)
				{
					b = 0;
					r = 0;
					g = 0;
				}
				testBox.Text = "R: " + r + " G: " + g + " B: " + b;
				xlabel.ForeColor = Color.FromArgb(r, g, b);
				if (b < 10)
				{
					colorDirection = !colorDirection;
					faderTime = 0;
				}
			}
			else
			{
				int b2 = xlabel.ForeColor.B;
				int r2 = xlabel.ForeColor.R;
				int g2 = xlabel.ForeColor.G;
				b2 += 5;
				g2 += 5;
				r2 += 5;
				if (b2 > 255)
				{
					b2 = 255;
					r2 = 255;
					g2 = 255;
				}
				testBox.Text = "R: " + r2 + " G: " + g2 + " B: " + b2;
				xlabel.ForeColor = Color.FromArgb(r2, g2, b2);
				if (b2 > 240)
				{
					colorDirection = !colorDirection;
					faderTime = 0;
				}
			}
		}
		faderTime++;
	}

	private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
	{
		recievedData = serialPort1.ReadExisting();
	}

	private void label5_Click(object sender, EventArgs e)
	{
	}

	private void timer2_Tick(object sender, EventArgs e)
	{
		updatePorts();
	}

	private void armedCheck_CheckedChanged(object sender, EventArgs e)
	{
		serialPort1.Write("i");
	}

	private void filterCheckbox_CheckedChanged(object sender, EventArgs e)
	{
		serialPort1.Write("f");
	}

	private void radioButton3_CheckedChanged(object sender, EventArgs e)
	{
		serialPort1.Write("cc");
	}

	private void dizzyRadio_CheckedChanged(object sender, EventArgs e)
	{
		serialPort1.Write("cd");
	}

	private void watedRadio_CheckedChanged(object sender, EventArgs e)
	{
		serialPort1.Write("cw");
	}

	private void toyotaCheckbox_CheckedChanged(object sender, EventArgs e)
	{
		toyotaBox.Enabled = toyotaCheckbox.Checked;
		if (!toyotaCheckbox.Checked)
		{
			serialPort1.Write("to");
		}
	}

	private void fourCylToyo_CheckedChanged(object sender, EventArgs e)
	{
		serialPort1.Write("tf");
	}

	private void sixCylToyo_CheckedChanged(object sender, EventArgs e)
	{
		serialPort1.Write("ts");
	}

	private void eightCylToyo_CheckedChanged(object sender, EventArgs e)
	{
		serialPort1.Write("te");
	}

	private void timer3_Tick(object sender, EventArgs e)
	{
		serialPort1.Write("r");
	}

	private void rpmButton_Click(object sender, EventArgs e)
	{
		serialPort1.Write("f");
	}

	private void button1_Click(object sender, EventArgs e)
	{
		serialPort1.Write("i");
	}

	private void rpmBox_TextChanged(object sender, EventArgs e)
	{
	}

	private void coilBox_SelectedIndexChanged(object sender, EventArgs e)
	{
		serialPort1.Write("c");
		string itemText = coilBox.GetItemText(coilBox.SelectedItem);
		serialPort1.Write(itemText);
	}

	private void checkBox1_CheckedChanged(object sender, EventArgs e)
	{
		serialPort1.Write("c");
		if (checkBox1.Checked)
		{
			serialPort1.Write("/");
		}
		else
		{
			serialPort1.Write("*");
		}
	}

	private void label12_Click(object sender, EventArgs e)
	{
	}

	private void toyotaBox_Enter(object sender, EventArgs e)
	{
	}

	private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
	{
		serialPort1.Write("c");
		string itemText = triggerBox.GetItemText(triggerBox.SelectedItem);
		serialPort1.Write(itemText);
	}

	private void label14_Click(object sender, EventArgs e)
	{
	}

	private void label13_Click(object sender, EventArgs e)
	{
	}

	private void armedVal_Click(object sender, EventArgs e)
	{
	}

	protected override void Dispose(bool disposing)
	{
		if (disposing && components != null)
		{
			components.Dispose();
		}
		base.Dispose(disposing);
	}

	private void InitializeComponent()
	{
		this.components = new System.ComponentModel.Container();
		this.COMbox = new System.Windows.Forms.ComboBox();
		this.label1 = new System.Windows.Forms.Label();
		this.connectBtn = new System.Windows.Forms.Button();
		this.rpmBox = new System.Windows.Forms.TextBox();
		this.label2 = new System.Windows.Forms.Label();
		this.label3 = new System.Windows.Forms.Label();
		this.hystBox = new System.Windows.Forms.TextBox();
		this.label4 = new System.Windows.Forms.Label();
		this.posBox = new System.Windows.Forms.TextBox();
		this.sendButton = new System.Windows.Forms.Button();
		this.timer1 = new System.Windows.Forms.Timer(this.components);
		this.groupBox1 = new System.Windows.Forms.GroupBox();
		this.testBox = new System.Windows.Forms.TextBox();
		this.xlabel = new System.Windows.Forms.Label();
		this.timer2 = new System.Windows.Forms.Timer(this.components);
		this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
		this.rpmRead = new System.Windows.Forms.Label();
		this.rpmLabel = new System.Windows.Forms.Label();
		this.potPosLabel = new System.Windows.Forms.Label();
		this.potPosVal = new System.Windows.Forms.Label();
		this.label6 = new System.Windows.Forms.Label();
		this.rpmHystVal = new System.Windows.Forms.Label();
		this.toyotaCheckbox = new System.Windows.Forms.CheckBox();
		this.cylCount = new System.Windows.Forms.Label();
		this.cylLabel = new System.Windows.Forms.Label();
		this.coilSetBox = new System.Windows.Forms.GroupBox();
		this.label12 = new System.Windows.Forms.Label();
		this.label5 = new System.Windows.Forms.Label();
		this.triggerBox = new System.Windows.Forms.ComboBox();
		this.checkBox1 = new System.Windows.Forms.CheckBox();
		this.coilBox = new System.Windows.Forms.ComboBox();
		this.extraBox = new System.Windows.Forms.GroupBox();
		this.rpmButton = new System.Windows.Forms.Button();
		this.clutchButton = new System.Windows.Forms.Button();
		this.coilVal = new System.Windows.Forms.Label();
		this.label7 = new System.Windows.Forms.Label();
		this.RPMfilterVal = new System.Windows.Forms.Label();
		this.label9 = new System.Windows.Forms.Label();
		this.toyotaVal = new System.Windows.Forms.Label();
		this.label8 = new System.Windows.Forms.Label();
		this.armedVal = new System.Windows.Forms.Label();
		this.label11 = new System.Windows.Forms.Label();
		this.toyotaBox = new System.Windows.Forms.GroupBox();
		this.eightCylToyo = new System.Windows.Forms.RadioButton();
		this.sixCylToyo = new System.Windows.Forms.RadioButton();
		this.fourCylToyo = new System.Windows.Forms.RadioButton();
		this.cutRPMval = new System.Windows.Forms.Label();
		this.label10 = new System.Windows.Forms.Label();
		this.States = new System.Windows.Forms.GroupBox();
		this.timer3 = new System.Windows.Forms.Timer(this.components);
		this.trigVal = new System.Windows.Forms.Label();
		this.label14 = new System.Windows.Forms.Label();
		this.groupBox1.SuspendLayout();
		this.coilSetBox.SuspendLayout();
		this.extraBox.SuspendLayout();
		this.toyotaBox.SuspendLayout();
		this.States.SuspendLayout();
		base.SuspendLayout();
		this.COMbox.FormattingEnabled = true;
		this.COMbox.Location = new System.Drawing.Point(504, 417);
		this.COMbox.Name = "COMbox";
		this.COMbox.Size = new System.Drawing.Size(121, 27);
		this.COMbox.TabIndex = 0;
		this.COMbox.Text = "[Select COM Port]";
		this.COMbox.SelectedIndexChanged += new System.EventHandler(COMlist_SelectedIndexChanged);
		this.label1.AutoSize = true;
		this.label1.Font = new System.Drawing.Font("Segoe UI", 12f, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, 0);
		this.label1.ForeColor = System.Drawing.Color.Snow;
		this.label1.Location = new System.Drawing.Point(414, 417);
		this.label1.Name = "label1";
		this.label1.Size = new System.Drawing.Size(105, 28);
		this.label1.TabIndex = 1;
		this.label1.Text = "Serial Port:";
		this.label1.Click += new System.EventHandler(label1_Click);
		this.connectBtn.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
		this.connectBtn.Location = new System.Drawing.Point(631, 415);
		this.connectBtn.Name = "connectBtn";
		this.connectBtn.Size = new System.Drawing.Size(75, 23);
		this.connectBtn.TabIndex = 2;
		this.connectBtn.Text = "Connect";
		this.connectBtn.UseVisualStyleBackColor = true;
		this.connectBtn.Click += new System.EventHandler(connectCom_Click);
		this.rpmBox.BackColor = System.Drawing.SystemColors.ButtonHighlight;
		this.rpmBox.Location = new System.Drawing.Point(129, 32);
		this.rpmBox.MaxLength = 4;
		this.rpmBox.Name = "rpmBox";
		this.rpmBox.Size = new System.Drawing.Size(138, 26);
		this.rpmBox.TabIndex = 4;
		this.rpmBox.TextChanged += new System.EventHandler(rpmBox_TextChanged);
		this.label2.AutoSize = true;
		this.label2.BackColor = System.Drawing.SystemColors.WindowText;
		this.label2.Font = new System.Drawing.Font("Segoe UI", 12f, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, 0);
		this.label2.ForeColor = System.Drawing.Color.Snow;
		this.label2.Location = new System.Drawing.Point(20, 32);
		this.label2.Name = "label2";
		this.label2.Size = new System.Drawing.Size(128, 28);
		this.label2.TabIndex = 5;
		this.label2.Text = "Desired RPM:";
		this.label3.AutoSize = true;
		this.label3.Font = new System.Drawing.Font("Segoe UI", 12f, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, 0);
		this.label3.ForeColor = System.Drawing.Color.Snow;
		this.label3.Location = new System.Drawing.Point(8, 69);
		this.label3.Name = "label3";
		this.label3.Size = new System.Drawing.Size(144, 28);
		this.label3.TabIndex = 7;
		this.label3.Text = "RPM Hysterisis:";
		this.hystBox.BackColor = System.Drawing.SystemColors.ButtonHighlight;
		this.hystBox.Location = new System.Drawing.Point(129, 69);
		this.hystBox.MaxLength = 4;
		this.hystBox.Name = "hystBox";
		this.hystBox.Size = new System.Drawing.Size(138, 26);
		this.hystBox.TabIndex = 6;
		this.hystBox.TextChanged += new System.EventHandler(hystBox_TextChanged);
		this.label4.AutoSize = true;
		this.label4.Font = new System.Drawing.Font("Segoe UI", 12f, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, 0);
		this.label4.ForeColor = System.Drawing.Color.Snow;
		this.label4.Location = new System.Drawing.Point(55, 108);
		this.label4.Name = "label4";
		this.label4.Size = new System.Drawing.Size(86, 28);
		this.label4.TabIndex = 9;
		this.label4.Text = "Position:";
		this.posBox.BackColor = System.Drawing.SystemColors.ButtonHighlight;
		this.posBox.Location = new System.Drawing.Point(129, 108);
		this.posBox.MaxLength = 1;
		this.posBox.Name = "posBox";
		this.posBox.Size = new System.Drawing.Size(138, 26);
		this.posBox.TabIndex = 8;
		this.posBox.TextChanged += new System.EventHandler(positionBox_TextChanged);
		this.sendButton.Enabled = false;
		this.sendButton.Location = new System.Drawing.Point(128, 157);
		this.sendButton.Name = "sendButton";
		this.sendButton.Size = new System.Drawing.Size(139, 26);
		this.sendButton.TabIndex = 10;
		this.sendButton.Text = "Send Values";
		this.sendButton.UseVisualStyleBackColor = true;
		this.sendButton.Click += new System.EventHandler(button3_Click);
		this.timer1.Interval = 50;
		this.timer1.Tick += new System.EventHandler(timer1_Tick);
		this.groupBox1.BackColor = System.Drawing.SystemColors.WindowText;
		this.groupBox1.Controls.Add(this.sendButton);
		this.groupBox1.Controls.Add(this.label4);
		this.groupBox1.Controls.Add(this.posBox);
		this.groupBox1.Controls.Add(this.label3);
		this.groupBox1.Controls.Add(this.hystBox);
		this.groupBox1.Controls.Add(this.label2);
		this.groupBox1.Controls.Add(this.rpmBox);
		this.groupBox1.Enabled = false;
		this.groupBox1.ForeColor = System.Drawing.Color.CornflowerBlue;
		this.groupBox1.Location = new System.Drawing.Point(23, 22);
		this.groupBox1.Name = "groupBox1";
		this.groupBox1.Size = new System.Drawing.Size(293, 203);
		this.groupBox1.TabIndex = 11;
		this.groupBox1.TabStop = false;
		this.groupBox1.Text = "Two Step Settings";
		this.testBox.Location = new System.Drawing.Point(400, 22);
		this.testBox.Name = "testBox";
		this.testBox.Size = new System.Drawing.Size(358, 26);
		this.testBox.TabIndex = 12;
		this.testBox.Visible = false;
		this.xlabel.AutoSize = true;
		this.xlabel.Font = new System.Drawing.Font("Times New Roman", 192f, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, 0);
		this.xlabel.ForeColor = System.Drawing.SystemColors.ButtonHighlight;
		this.xlabel.Location = new System.Drawing.Point(549, 203);
		this.xlabel.Name = "xlabel";
		this.xlabel.Size = new System.Drawing.Size(379, 354);
		this.xlabel.TabIndex = 13;
		this.xlabel.Text = "X";
		this.xlabel.Click += new System.EventHandler(label5_Click);
		this.timer2.Enabled = true;
		this.timer2.Interval = 1000;
		this.timer2.Tick += new System.EventHandler(timer2_Tick);
		this.rpmRead.AutoSize = true;
		this.rpmRead.Font = new System.Drawing.Font("Segoe UI Semibold", 48f, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, 0);
		this.rpmRead.Location = new System.Drawing.Point(532, 59);
		this.rpmRead.Name = "rpmRead";
		this.rpmRead.Size = new System.Drawing.Size(91, 106);
		this.rpmRead.TabIndex = 14;
		this.rpmRead.Text = "0";
		this.rpmLabel.AutoSize = true;
		this.rpmLabel.Font = new System.Drawing.Font("Segoe UI Semibold", 48f, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, 0);
		this.rpmLabel.Location = new System.Drawing.Point(357, 61);
		this.rpmLabel.Name = "rpmLabel";
		this.rpmLabel.Size = new System.Drawing.Size(230, 106);
		this.rpmLabel.TabIndex = 15;
		this.rpmLabel.Text = "RPM:";
		this.potPosLabel.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.potPosLabel.AutoSize = true;
		this.potPosLabel.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.potPosLabel.ForeColor = System.Drawing.SystemColors.Info;
		this.potPosLabel.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.potPosLabel.Location = new System.Drawing.Point(37, 17);
		this.potPosLabel.Name = "potPosLabel";
		this.potPosLabel.Size = new System.Drawing.Size(120, 28);
		this.potPosLabel.TabIndex = 16;
		this.potPosLabel.Text = "Pot Position:";
		this.potPosLabel.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.potPosVal.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.potPosVal.AutoSize = true;
		this.potPosVal.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.potPosVal.ForeColor = System.Drawing.SystemColors.Info;
		this.potPosVal.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.potPosVal.Location = new System.Drawing.Point(135, 17);
		this.potPosVal.Name = "potPosVal";
		this.potPosVal.Size = new System.Drawing.Size(42, 28);
		this.potPosVal.TabIndex = 17;
		this.potPosVal.Text = "aaa";
		this.potPosVal.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.label6.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.label6.AutoSize = true;
		this.label6.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.label6.ForeColor = System.Drawing.SystemColors.Info;
		this.label6.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.label6.Location = new System.Drawing.Point(14, 59);
		this.label6.Name = "label6";
		this.label6.Size = new System.Drawing.Size(144, 28);
		this.label6.TabIndex = 18;
		this.label6.Text = "RPM Hysterisis:";
		this.label6.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.rpmHystVal.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.rpmHystVal.AutoSize = true;
		this.rpmHystVal.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.rpmHystVal.ForeColor = System.Drawing.SystemColors.Info;
		this.rpmHystVal.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.rpmHystVal.Location = new System.Drawing.Point(135, 59);
		this.rpmHystVal.Name = "rpmHystVal";
		this.rpmHystVal.Size = new System.Drawing.Size(42, 28);
		this.rpmHystVal.TabIndex = 19;
		this.rpmHystVal.Text = "aaa";
		this.rpmHystVal.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.toyotaCheckbox.AutoSize = true;
		this.toyotaCheckbox.Font = new System.Drawing.Font("Segoe UI", 10f);
		this.toyotaCheckbox.ForeColor = System.Drawing.SystemColors.Info;
		this.toyotaCheckbox.Location = new System.Drawing.Point(6, 76);
		this.toyotaCheckbox.Name = "toyotaCheckbox";
		this.toyotaCheckbox.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
		this.toyotaCheckbox.Size = new System.Drawing.Size(71, 27);
		this.toyotaCheckbox.TabIndex = 23;
		this.toyotaCheckbox.Text = ":Toyo";
		this.toyotaCheckbox.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
		this.toyotaCheckbox.UseVisualStyleBackColor = true;
		this.toyotaCheckbox.CheckedChanged += new System.EventHandler(toyotaCheckbox_CheckedChanged);
		this.cylCount.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.cylCount.AutoSize = true;
		this.cylCount.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.cylCount.ForeColor = System.Drawing.SystemColors.Info;
		this.cylCount.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.cylCount.Location = new System.Drawing.Point(135, 83);
		this.cylCount.Name = "cylCount";
		this.cylCount.Size = new System.Drawing.Size(42, 28);
		this.cylCount.TabIndex = 25;
		this.cylCount.Text = "aaa";
		this.cylCount.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.cylLabel.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.cylLabel.AutoSize = true;
		this.cylLabel.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.cylLabel.ForeColor = System.Drawing.SystemColors.Info;
		this.cylLabel.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.cylLabel.Location = new System.Drawing.Point(52, 83);
		this.cylLabel.Name = "cylLabel";
		this.cylLabel.Size = new System.Drawing.Size(100, 28);
		this.cylLabel.TabIndex = 24;
		this.cylLabel.Text = "Cyl Mode:";
		this.cylLabel.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.coilSetBox.Controls.Add(this.label12);
		this.coilSetBox.Controls.Add(this.label5);
		this.coilSetBox.Controls.Add(this.triggerBox);
		this.coilSetBox.Controls.Add(this.checkBox1);
		this.coilSetBox.Controls.Add(this.coilBox);
		this.coilSetBox.Enabled = false;
		this.coilSetBox.ForeColor = System.Drawing.SystemColors.MenuHighlight;
		this.coilSetBox.Location = new System.Drawing.Point(23, 243);
		this.coilSetBox.Name = "coilSetBox";
		this.coilSetBox.Size = new System.Drawing.Size(194, 118);
		this.coilSetBox.TabIndex = 26;
		this.coilSetBox.TabStop = false;
		this.coilSetBox.Text = "Coil Settings";
		this.label12.AutoSize = true;
		this.label12.Location = new System.Drawing.Point(114, 22);
		this.label12.Name = "label12";
		this.label12.Size = new System.Drawing.Size(83, 19);
		this.label12.TabIndex = 44;
		this.label12.Text = "Trigger Type";
		this.label12.Click += new System.EventHandler(label12_Click);
		this.label5.AutoSize = true;
		this.label5.Location = new System.Drawing.Point(25, 24);
		this.label5.Name = "label5";
		this.label5.Size = new System.Drawing.Size(64, 19);
		this.label5.TabIndex = 43;
		this.label5.Text = "Coil Type";
		this.triggerBox.FormattingEnabled = true;
		this.triggerBox.Items.AddRange(new object[11]
		{
			"a", "b", "c", "d", "e", "f", "g", "h", "i", "j",
			"k"
		});
		this.triggerBox.Location = new System.Drawing.Point(118, 47);
		this.triggerBox.MaxDropDownItems = 16;
		this.triggerBox.MaxLength = 1;
		this.triggerBox.Name = "triggerBox";
		this.triggerBox.Size = new System.Drawing.Size(49, 27);
		this.triggerBox.TabIndex = 42;
		this.triggerBox.Text = "a";
		this.triggerBox.SelectedIndexChanged += new System.EventHandler(comboBox1_SelectedIndexChanged);
		this.checkBox1.AutoSize = true;
		this.checkBox1.ForeColor = System.Drawing.SystemColors.HighlightText;
		this.checkBox1.Location = new System.Drawing.Point(46, 86);
		this.checkBox1.Name = "checkBox1";
		this.checkBox1.Size = new System.Drawing.Size(102, 23);
		this.checkBox1.TabIndex = 41;
		this.checkBox1.Text = "Divide RPM";
		this.checkBox1.UseVisualStyleBackColor = true;
		this.checkBox1.Visible = false;
		this.checkBox1.CheckedChanged += new System.EventHandler(checkBox1_CheckedChanged);
		this.coilBox.FormattingEnabled = true;
		this.coilBox.Items.AddRange(new object[9] { "1", "2", "3", "4", "5", "6", "7", "8", "9" });
		this.coilBox.Location = new System.Drawing.Point(25, 47);
		this.coilBox.MaxDropDownItems = 16;
		this.coilBox.MaxLength = 1;
		this.coilBox.Name = "coilBox";
		this.coilBox.Size = new System.Drawing.Size(49, 27);
		this.coilBox.TabIndex = 40;
		this.coilBox.Text = "1";
		this.coilBox.SelectedIndexChanged += new System.EventHandler(coilBox_SelectedIndexChanged);
		this.extraBox.Controls.Add(this.rpmButton);
		this.extraBox.Controls.Add(this.clutchButton);
		this.extraBox.Controls.Add(this.toyotaCheckbox);
		this.extraBox.Enabled = false;
		this.extraBox.ForeColor = System.Drawing.SystemColors.MenuHighlight;
		this.extraBox.Location = new System.Drawing.Point(228, 243);
		this.extraBox.Name = "extraBox";
		this.extraBox.Size = new System.Drawing.Size(88, 118);
		this.extraBox.TabIndex = 27;
		this.extraBox.TabStop = false;
		this.extraBox.Text = "Extra";
		this.rpmButton.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
		this.rpmButton.Location = new System.Drawing.Point(6, 54);
		this.rpmButton.Name = "rpmButton";
		this.rpmButton.Size = new System.Drawing.Size(76, 23);
		this.rpmButton.TabIndex = 40;
		this.rpmButton.Text = "RPM Filter";
		this.rpmButton.UseVisualStyleBackColor = true;
		this.rpmButton.Click += new System.EventHandler(rpmButton_Click);
		this.clutchButton.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
		this.clutchButton.Location = new System.Drawing.Point(6, 25);
		this.clutchButton.Name = "clutchButton";
		this.clutchButton.Size = new System.Drawing.Size(76, 23);
		this.clutchButton.TabIndex = 41;
		this.clutchButton.Text = "Clutch Input";
		this.clutchButton.UseVisualStyleBackColor = true;
		this.clutchButton.Click += new System.EventHandler(button1_Click);
		this.coilVal.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.coilVal.AutoSize = true;
		this.coilVal.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.coilVal.ForeColor = System.Drawing.SystemColors.Info;
		this.coilVal.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.coilVal.Location = new System.Drawing.Point(135, 107);
		this.coilVal.Name = "coilVal";
		this.coilVal.Size = new System.Drawing.Size(42, 28);
		this.coilVal.TabIndex = 29;
		this.coilVal.Text = "aaa";
		this.coilVal.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.label7.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.label7.AutoSize = true;
		this.label7.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.label7.ForeColor = System.Drawing.SystemColors.Info;
		this.label7.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.label7.Location = new System.Drawing.Point(44, 107);
		this.label7.Name = "label7";
		this.label7.Size = new System.Drawing.Size(108, 28);
		this.label7.TabIndex = 28;
		this.label7.Text = "Coil Power:";
		this.label7.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.RPMfilterVal.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.RPMfilterVal.AutoSize = true;
		this.RPMfilterVal.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.RPMfilterVal.ForeColor = System.Drawing.SystemColors.Info;
		this.RPMfilterVal.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.RPMfilterVal.Location = new System.Drawing.Point(135, 131);
		this.RPMfilterVal.Name = "RPMfilterVal";
		this.RPMfilterVal.Size = new System.Drawing.Size(42, 28);
		this.RPMfilterVal.TabIndex = 31;
		this.RPMfilterVal.Text = "aaa";
		this.RPMfilterVal.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.label9.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.label9.AutoSize = true;
		this.label9.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.label9.ForeColor = System.Drawing.SystemColors.Info;
		this.label9.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.label9.Location = new System.Drawing.Point(46, 131);
		this.label9.Name = "label9";
		this.label9.Size = new System.Drawing.Size(106, 28);
		this.label9.TabIndex = 30;
		this.label9.Text = "RPM Filter:";
		this.label9.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.toyotaVal.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.toyotaVal.AutoSize = true;
		this.toyotaVal.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.toyotaVal.ForeColor = System.Drawing.SystemColors.Info;
		this.toyotaVal.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.toyotaVal.Location = new System.Drawing.Point(135, 155);
		this.toyotaVal.Name = "toyotaVal";
		this.toyotaVal.Size = new System.Drawing.Size(42, 28);
		this.toyotaVal.TabIndex = 33;
		this.toyotaVal.Text = "aaa";
		this.toyotaVal.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.label8.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.label8.AutoSize = true;
		this.label8.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.label8.ForeColor = System.Drawing.SystemColors.Info;
		this.label8.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.label8.Location = new System.Drawing.Point(29, 155);
		this.label8.Name = "label8";
		this.label8.Size = new System.Drawing.Size(132, 28);
		this.label8.TabIndex = 32;
		this.label8.Text = "Toyota Mode:";
		this.label8.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.armedVal.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.armedVal.AutoSize = true;
		this.armedVal.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.armedVal.ForeColor = System.Drawing.SystemColors.Info;
		this.armedVal.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.armedVal.Location = new System.Drawing.Point(135, 179);
		this.armedVal.Name = "armedVal";
		this.armedVal.Size = new System.Drawing.Size(42, 28);
		this.armedVal.TabIndex = 35;
		this.armedVal.Text = "aaa";
		this.armedVal.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.armedVal.Click += new System.EventHandler(armedVal_Click);
		this.label11.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.label11.AutoSize = true;
		this.label11.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.label11.ForeColor = System.Drawing.SystemColors.Info;
		this.label11.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.label11.Location = new System.Drawing.Point(16, 176);
		this.label11.Name = "label11";
		this.label11.Size = new System.Drawing.Size(142, 28);
		this.label11.TabIndex = 34;
		this.label11.Text = "System Armed:";
		this.label11.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.toyotaBox.Controls.Add(this.eightCylToyo);
		this.toyotaBox.Controls.Add(this.sixCylToyo);
		this.toyotaBox.Controls.Add(this.fourCylToyo);
		this.toyotaBox.Enabled = false;
		this.toyotaBox.ForeColor = System.Drawing.SystemColors.MenuHighlight;
		this.toyotaBox.Location = new System.Drawing.Point(23, 367);
		this.toyotaBox.Name = "toyotaBox";
		this.toyotaBox.Size = new System.Drawing.Size(293, 70);
		this.toyotaBox.TabIndex = 36;
		this.toyotaBox.TabStop = false;
		this.toyotaBox.Text = "Toyota Modes";
		this.toyotaBox.Enter += new System.EventHandler(toyotaBox_Enter);
		this.eightCylToyo.AutoSize = true;
		this.eightCylToyo.ForeColor = System.Drawing.SystemColors.Info;
		this.eightCylToyo.Location = new System.Drawing.Point(191, 32);
		this.eightCylToyo.Name = "eightCylToyo";
		this.eightCylToyo.Size = new System.Drawing.Size(92, 23);
		this.eightCylToyo.TabIndex = 2;
		this.eightCylToyo.TabStop = true;
		this.eightCylToyo.Text = "8 Cylinder";
		this.eightCylToyo.UseVisualStyleBackColor = true;
		this.eightCylToyo.CheckedChanged += new System.EventHandler(eightCylToyo_CheckedChanged);
		this.sixCylToyo.AutoSize = true;
		this.sixCylToyo.ForeColor = System.Drawing.SystemColors.Info;
		this.sixCylToyo.Location = new System.Drawing.Point(102, 32);
		this.sixCylToyo.Name = "sixCylToyo";
		this.sixCylToyo.Size = new System.Drawing.Size(92, 23);
		this.sixCylToyo.TabIndex = 1;
		this.sixCylToyo.TabStop = true;
		this.sixCylToyo.Text = "6 Cylinder";
		this.sixCylToyo.UseVisualStyleBackColor = true;
		this.sixCylToyo.CheckedChanged += new System.EventHandler(sixCylToyo_CheckedChanged);
		this.fourCylToyo.AutoSize = true;
		this.fourCylToyo.ForeColor = System.Drawing.SystemColors.Info;
		this.fourCylToyo.Location = new System.Drawing.Point(13, 32);
		this.fourCylToyo.Name = "fourCylToyo";
		this.fourCylToyo.Size = new System.Drawing.Size(92, 23);
		this.fourCylToyo.TabIndex = 0;
		this.fourCylToyo.TabStop = true;
		this.fourCylToyo.Text = "4 Cylinder";
		this.fourCylToyo.UseVisualStyleBackColor = true;
		this.fourCylToyo.CheckedChanged += new System.EventHandler(fourCylToyo_CheckedChanged);
		this.cutRPMval.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.cutRPMval.AutoSize = true;
		this.cutRPMval.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.cutRPMval.ForeColor = System.Drawing.SystemColors.Info;
		this.cutRPMval.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.cutRPMval.Location = new System.Drawing.Point(135, 38);
		this.cutRPMval.Name = "cutRPMval";
		this.cutRPMval.Size = new System.Drawing.Size(42, 28);
		this.cutRPMval.TabIndex = 38;
		this.cutRPMval.Text = "aaa";
		this.cutRPMval.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.label10.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.label10.AutoSize = true;
		this.label10.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.label10.ForeColor = System.Drawing.SystemColors.Info;
		this.label10.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.label10.Location = new System.Drawing.Point(36, 38);
		this.label10.Name = "label10";
		this.label10.Size = new System.Drawing.Size(120, 28);
		this.label10.TabIndex = 37;
		this.label10.Text = "2-Step RPM:";
		this.label10.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.States.Controls.Add(this.trigVal);
		this.States.Controls.Add(this.label14);
		this.States.Controls.Add(this.cutRPMval);
		this.States.Controls.Add(this.label10);
		this.States.Controls.Add(this.armedVal);
		this.States.Controls.Add(this.label11);
		this.States.Controls.Add(this.toyotaVal);
		this.States.Controls.Add(this.label8);
		this.States.Controls.Add(this.RPMfilterVal);
		this.States.Controls.Add(this.label9);
		this.States.Controls.Add(this.coilVal);
		this.States.Controls.Add(this.label7);
		this.States.Controls.Add(this.cylCount);
		this.States.Controls.Add(this.cylLabel);
		this.States.Controls.Add(this.rpmHystVal);
		this.States.Controls.Add(this.label6);
		this.States.Controls.Add(this.potPosVal);
		this.States.Controls.Add(this.potPosLabel);
		this.States.ForeColor = System.Drawing.SystemColors.MenuHighlight;
		this.States.Location = new System.Drawing.Point(333, 165);
		this.States.Name = "States";
		this.States.Size = new System.Drawing.Size(183, 246);
		this.States.TabIndex = 39;
		this.States.TabStop = false;
		this.States.Text = "States";
		this.timer3.Tick += new System.EventHandler(timer3_Tick);
		this.trigVal.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.trigVal.AutoSize = true;
		this.trigVal.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.trigVal.ForeColor = System.Drawing.SystemColors.Info;
		this.trigVal.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.trigVal.Location = new System.Drawing.Point(135, 202);
		this.trigVal.Name = "trigVal";
		this.trigVal.Size = new System.Drawing.Size(42, 28);
		this.trigVal.TabIndex = 40;
		this.trigVal.Text = "aaa";
		this.trigVal.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.trigVal.Click += new System.EventHandler(label13_Click);
		this.label14.Anchor = System.Windows.Forms.AnchorStyles.Left;
		this.label14.AutoSize = true;
		this.label14.Font = new System.Drawing.Font("Segoe UI", 12f);
		this.label14.ForeColor = System.Drawing.SystemColors.Info;
		this.label14.ImageAlign = System.Drawing.ContentAlignment.TopRight;
		this.label14.Location = new System.Drawing.Point(52, 202);
		this.label14.Name = "label14";
		this.label14.Size = new System.Drawing.Size(94, 28);
		this.label14.TabIndex = 39;
		this.label14.Text = "Trig Type:";
		this.label14.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
		this.label14.Click += new System.EventHandler(label14_Click);
		base.AutoScaleDimensions = new System.Drawing.SizeF(8f, 19f);
		base.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
		this.BackColor = System.Drawing.SystemColors.WindowText;
		base.ClientSize = new System.Drawing.Size(800, 450);
		base.Controls.Add(this.States);
		base.Controls.Add(this.toyotaBox);
		base.Controls.Add(this.extraBox);
		base.Controls.Add(this.coilSetBox);
		base.Controls.Add(this.rpmLabel);
		base.Controls.Add(this.rpmRead);
		base.Controls.Add(this.testBox);
		base.Controls.Add(this.groupBox1);
		base.Controls.Add(this.connectBtn);
		base.Controls.Add(this.label1);
		base.Controls.Add(this.COMbox);
		base.Controls.Add(this.xlabel);
		this.Font = new System.Drawing.Font("Segoe UI", 8.25f, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, 0);
		this.ForeColor = System.Drawing.SystemColors.ControlLightLight;
		base.Name = "Form1";
		this.Text = "AlphaX TwoStep Box Configurator";
		base.Load += new System.EventHandler(Form1_Load);
		this.groupBox1.ResumeLayout(false);
		this.groupBox1.PerformLayout();
		this.coilSetBox.ResumeLayout(false);
		this.coilSetBox.PerformLayout();
		this.extraBox.ResumeLayout(false);
		this.extraBox.PerformLayout();
		this.toyotaBox.ResumeLayout(false);
		this.toyotaBox.PerformLayout();
		this.States.ResumeLayout(false);
		this.States.PerformLayout();
		base.ResumeLayout(false);
		base.PerformLayout();
	}
}
