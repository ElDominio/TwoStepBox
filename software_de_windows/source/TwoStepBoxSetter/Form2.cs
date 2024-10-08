using System;
using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;

namespace TwoStepBoxSetter;

public class Form2 : Form
{
	private IContainer components;

	private BackgroundWorker backgroundWorker1;

	private Label label1;

	private Button button1;

	public Form2()
	{
		InitializeComponent();
	}

	private void button1_Click(object sender, EventArgs e)
	{
		Close();
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
		this.backgroundWorker1 = new System.ComponentModel.BackgroundWorker();
		this.label1 = new System.Windows.Forms.Label();
		this.button1 = new System.Windows.Forms.Button();
		base.SuspendLayout();
		this.label1.AutoSize = true;
		this.label1.Font = new System.Drawing.Font("Segoe UI", 15.75f, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, 0);
		this.label1.Location = new System.Drawing.Point(21, 37);
		this.label1.Name = "label1";
		this.label1.Size = new System.Drawing.Size(254, 30);
		this.label1.TabIndex = 0;
		this.label1.Text = "All values have been sent!";
		this.button1.Font = new System.Drawing.Font("Segoe UI Emoji", 12f, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, 0);
		this.button1.Location = new System.Drawing.Point(55, 96);
		this.button1.Name = "button1";
		this.button1.Size = new System.Drawing.Size(187, 32);
		this.button1.TabIndex = 1;
		this.button1.Text = "Flame on \ud83d\udd25";
		this.button1.UseVisualStyleBackColor = true;
		this.button1.Click += new System.EventHandler(button1_Click);
		base.AutoScaleDimensions = new System.Drawing.SizeF(6f, 13f);
		base.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
		base.ClientSize = new System.Drawing.Size(296, 149);
		base.Controls.Add(this.button1);
		base.Controls.Add(this.label1);
		base.Name = "Form2";
		this.Text = "Confirmation";
		base.ResumeLayout(false);
		base.PerformLayout();
	}
}
