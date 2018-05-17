package com.example.arturas.helloworld;

import android.support.v7.app.AppCompatActivity;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;
import android.os.Bundle;



public class MainActivity extends AppCompatActivity {
    SeekBar myControl;
    TextView myTextView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        myControl = (SeekBar) findViewById(R.id.seek1);

        myTextView = (TextView) findViewById(R.id.textView01);
        myTextView.setText("Android Pls Work");

        setMyControlListener();
    }

    private void setMyControlListener() {
        myControl.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = progress;
                myTextView.setText("The value is: "+progress);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }


}


