package x.tracker;

import android.app.ActionBar;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.Typeface;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.MediaPlayer;
import android.net.Uri;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.VideoView;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.Formatter;
import java.util.Vector;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.navigation.fragment.NavHostFragment;
import x.tracker.databinding.FragmentFirstBinding;

import static android.content.Context.MODE_APPEND;
import static android.content.Context.TELECOM_SERVICE;

public class FirstFragment extends Fragment implements View.OnTouchListener, SensorEventListener {

    private FragmentFirstBinding binding;
    SurfaceView video;
    Bitmap videoBitmap;
    Canvas videoCanvas;
    ClientThread client;
// in case frames come in faster than we can draw them
    static boolean busy = false;

// keypoints for the next frame
    static int animals = 0;
    static final int MAX_ANIMALS = 2;
    static final int BODY_PARTS = 25;
// input coordinates
    static int[] keypoints = new int[MAX_ANIMALS * BODY_PARTS * 2];
// screen coordinates
    static int[] keypoint_cache = new int[MAX_ANIMALS * BODY_PARTS * 2];
    static float fps = 0;

// the body parts as defined in
// src/openpose/pose/poseParameters.cpp: POSE_BODY_25_BODY_PARTS
    static final int MODEL_NOSE = 0;
    static final int MODEL_NECK = 1;
    static final int MODEL_BUTT = 8;
    static final int MODEL_HIP1 = 12;
    static final int MODEL_HIP2 = 9;
    static final int MODEL_KNEE1 = 13;
    static final int MODEL_KNEE2 = 10;
    static final int MODEL_ELBOW1 = 6;
    static final int MODEL_ELBOW2 = 3;
    static final int MODEL_SHOULDER1 = 5;
    static final int MODEL_SHOULDER2 = 2;
    static final int MODEL_ANKLE1 = 14;
    static final int MODEL_ANKLE2 = 11;
    static final int MODEL_WRIST1 = 7;
    static final int MODEL_WRIST2 = 4;
    static final int MODEL_EYE2 = 15;
    static final int MODEL_EYE1 = 16;
    static final int MODEL_EAR2 = 17;
    static final int MODEL_EAR1 = 18;
    static final int MODEL_BIGTOE1 = 19;
    static final int MODEL_SMALLTOE1 = 20;
    static final int MODEL_HEEL1 = 21;
    static final int MODEL_BIGTOE2 = 22;
    static final int MODEL_SMALLTOE2 = 23;
    static final int MODEL_HEEL2 = 24;


// from include/openpose/pose/poseParametersRender.hpp
    static final int[] colors =
    {
        0x80ff0055, // NOSE
        0x80ff0000, // NECK
        0x80ff5500, // SHOULDER2
        0x80ffaa00, // ELBOW2 orange
        0x80ffff00, // WRIST2
        0x80aaff00, // SHOULDER1 light green
        0x8055ff00, // ELBOW1
        0x8000ff00, // WRIST1
        0x80ff0000, // BUTT
        0x8000ff55, // HIP2
        0x8000ffaa,  // KNEE2
        0x8000ffff, // ANKLE2
        0x8000aaff, // HIP1
        0x800055ff, // KNEE1
        0x800000ff, // ANKLE1
        0x80ff00aa, // REYE
        0x80aa00ff, // LEYE
        0x80ff00ff, // LEAR
        0x805500ff, // REAR
        0x800000ff, 
        0x800000ff, 
        0x800000ff, 
        0x8000ffff, 
        0x8000ffff, 
        0x8000ffff, 
    };



    // size of the encoded video
    static final int W = 640;
    static final int H = 360;

    static float dstX;
    static float dstY;
    static float dstH;
    static float dstW;

    static final int OFF = -1;
    static final int STARTUP  = 0;
    static final int CONFIGURING = 1;
    static final int TRACKING = 2;
    static int currentOperation = OFF;
    int prevOperation = OFF;

    int pan;
    int tilt;
    int start_pan;
    int start_tilt;
    int pan_sign;
    int tilt_sign;
    int lens;
    static boolean landscape = false;
    static boolean prevLandscape = false;

// error codes
    final int VIDEO_DEVICE_ERROR = 1;
    final int VIDEO_BUFFER_ERROR = 2;
    final int SERVO_ERROR = 4;
    final int CAM_ENUM_ERROR = 8;
    final int CAM_STARTING_ERROR = 16;

    int errors;

    float accelX = 0;
    float accelY = 0;
    final float ACCEL_BANDWIDTH = (float)0.2;

    Vector<Button> buttons = new Vector();
    Vector<Text> texts = new Vector();
    Button landscapeButton;
    Button lensButton;
    Button tiltButton;
    Button panButton;
    Text panText;
    Text tiltText;
    Text videoDeviceError;
    Text videoIoctlError;
    Text camStartingError;
    Text servoError;
    Text usbError;
    Text fpsText;
    static final int MARGIN = 40;
    static final int ARROW_MARGIN = 20;
    static final int TEXT_SIZE = 40;

    @Override
    public View onCreateView(
            LayoutInflater inflater, ViewGroup container,
            Bundle savedInstanceState
    ) {
        // landscape mode with no status bar
//        getActivity().setRequestedOrientation(
//                ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
//        View decorView = getActivity().getWindow().getDecorView();
//// Hide the status bar in landscape mode.
//        int uiOptions = View.SYSTEM_UI_FLAG_FULLSCREEN;
//        decorView.setSystemUiVisibility(uiOptions);

//        ActionBar actionBar = getActivity().getActionBar();
//        actionBar.hide();

        binding = FragmentFirstBinding.inflate(inflater, container, false);
        video = binding.surfaceView2;





      return binding.getRoot();

    }

    public void onViewCreated(@NonNull View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        binding.buttonFirst.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                NavHostFragment.findNavController(FirstFragment.this)
                        .navigate(R.id.action_FirstFragment_to_SecondFragment);
            }
        });
        // attach the listener
        video.setOnTouchListener(this);

        SensorManager sensorManager = (SensorManager) getActivity().getSystemService(Context.SENSOR_SERVICE);
        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);


        videoBitmap = Bitmap.createBitmap(W, H, Bitmap.Config.ARGB_8888);
        videoCanvas = new Canvas();
        videoCanvas.setBitmap(videoBitmap);


        new Thread(client = new ClientThread(this)).start();

    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }

    // wait for the surface view to be created
    boolean gotIt = false;
    public void waitForSurface()
    {
        while(!gotIt)
        {
            try {
                Thread.sleep(100L);
            } catch (Exception e) {
                e.printStackTrace();
            }

            getActivity().runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Canvas canvas = video.getHolder().lockCanvas();
                        if(canvas != null) {



                            video.getHolder().unlockCanvasAndPost(canvas);
                            gotIt = true;
                        }
                    }
                }
            );
        }
    }


    public void drawStatus(String text)
    {
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Canvas canvas = video.getHolder().lockCanvas();
                if(canvas != null) {
                    Paint p = new Paint();
                    p.setColor(Color.GREEN);
                    p.setStyle(Paint.Style.FILL);
                    p.setTypeface(Typeface.create("SansSerif", Typeface.BOLD));
                    p.setTextSize(TEXT_SIZE);
                    Rect text_size = new Rect();
                    p.getTextBounds(text,
                               0,
                               text.length(),
                               text_size);
                    if(landscape) {
                        // make a temporary canvas for rotating the text
                        Bitmap temp2 = Bitmap.createBitmap(text_size.width(), text_size.height(), Bitmap.Config.ARGB_8888);
                        Canvas temp = new Canvas(temp2);
                        temp.drawText(text, 0, temp.getHeight() - text_size.bottom, p);

                        // rotate & draw it
                        Matrix matrix = new Matrix();
                        matrix.reset();
                        matrix.postTranslate(-temp.getWidth() / 2, -temp.getHeight() / 2); // Centers image
                        matrix.postRotate(90);
                        matrix.postTranslate(canvas.getWidth() / 2, canvas.getHeight() / 2);

                        p.setColor(Color.BLACK);
                        canvas.drawRect(new Rect(0, 0, canvas.getWidth(), canvas.getHeight()), p);
                        canvas.drawBitmap(temp2,
                                matrix,
                                p);
                    }
                    else
                    {
                        p.setColor(Color.BLACK);
                        canvas.drawRect(new Rect(0,
                                canvas.getHeight() / 2 - text_size.height() / 2,
                                canvas.getWidth(),
                                canvas.getHeight() / 2 + text_size.height() / 2), p);
                        p.setColor(Color.GREEN);
                        canvas.drawText(text,
                                canvas.getWidth() / 2 - text_size.width() / 2,
                                canvas.getHeight() / 2 + text_size.height() / 2 - text_size.bottom,
                                p);

                    }

//Log.i("FirstFragment", "drawStatus " + text);
                    video.getHolder().unlockCanvasAndPost(canvas);
                }
                else
                {
                    Log.i("drawStatus", "No canvas");
                }
            }
        });
    }

    public void drawVideo(Bitmap bitmap) {
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Canvas canvas = video.getHolder().lockCanvas();

                if(canvas != null) {
                    Paint p = new Paint();
// must convert to a software bitmap for draw()
                    Bitmap softBitmap = bitmap.copy(Bitmap.Config.ARGB_8888, false);
                    videoCanvas.drawBitmap(softBitmap, 0, 0, p);


                    drawGUI(canvas);

                    video.getHolder().unlockCanvasAndPost(canvas);
                    busy = false;
                }
            }
        });
    }

    boolean updateErrors()
    {
        boolean needRedraw = false;
        needRedraw |= videoDeviceError.setHidden((errors & VIDEO_DEVICE_ERROR) == 0);
        needRedraw |= videoIoctlError.setHidden((errors & VIDEO_BUFFER_ERROR) == 0);
        needRedraw |= servoError.setHidden((errors & SERVO_ERROR) == 0);
        needRedraw |= usbError.setHidden((errors & CAM_ENUM_ERROR) == 0);
        needRedraw |= camStartingError.setHidden((errors & CAM_STARTING_ERROR) == 0);
        return needRedraw;
    }

    public void updateValues() {
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                boolean needRedraw = false;
                if (panButton != null) {
                    needRedraw |= panButton.updateText(String.valueOf(pan_sign));
                }
                if (tiltButton != null) {
                    needRedraw |= tiltButton.updateText(String.valueOf(tilt_sign));
                }
                if(panText != null)
                {
                    needRedraw |= panText.updateText(getPanText());
                }
                if(tiltText != null)
                {
                    needRedraw |= tiltText.updateText(getTiltText());
                }
                if(lensButton != null)
                {
                    needRedraw |= lensButton.updateText(lensText());
                }
                if(landscapeButton != null)
                {
                    needRedraw |= landscapeButton.updateText(landscapeText());
                }

                needRedraw |= updateErrors();

                if(needRedraw) {
                    Canvas canvas = video.getHolder().lockCanvas();
                    if (canvas != null) {
                        drawGUI(canvas);
                        video.getHolder().unlockCanvasAndPost(canvas);
                    }
                }
            }
        });
    }

    public void changeOperationSync()
    {
        buttons.clear();
        texts.clear();
        panButton = null;
        tiltButton = null;
        panText = null;
        tiltText = null;
        lensButton = null;
        landscapeButton = null;



        Canvas canvas = video.getHolder().lockCanvas();

        if (canvas != null) {
            int x, y;
            Rect size = Text.calculateSize("X");
            if(landscape) {
                x = canvas.getWidth() / 4;
                y = canvas.getHeight() / 2;
            }
            else
            {
                x = MARGIN;
                y = size.height() / 2 + MARGIN;
            }

            if(landscape)
                fpsText = new Text(canvas.getWidth() - size.width() - MARGIN,
                    MARGIN,
                    "");
            else
                fpsText = new Text(MARGIN,
                    MARGIN + size.height(),
                    "");

            usbError = new Text(x, y, "VIDEO USB NOT FOUND");
            usbError.color = Color.RED;
//             if(landscape) {
//                 x += usbError.getW();
//             }
//             else
//             {
//                 y += usbError.getH();
//             }

            videoDeviceError = new Text(x, y, "/DEV/VIDEO* NOT FOUND");
            videoDeviceError.color = Color.RED;
//             if(landscape) {
//                 x += usbError.getW();
//             }
//             else
//             {
//                 y += usbError.getH();
//             }

            videoIoctlError = new Text(x, y, "VIDEO IOCTL FAILED");
            videoIoctlError.color = Color.RED;
//             if(landscape) {
//                 x += usbError.getW();
//             }
//             else
//             {
//                 y += usbError.getH();
//             }


            camStartingError = new Text(x, y, "VIDEO STARTING");
            camStartingError.color = Color.RED;
            if(landscape) {
                x += usbError.getW();
            }
            else
            {
                y += usbError.getH();
            }

            servoError = new Text(x, y, "SERVO DEVICE NOT FOUND");
            servoError.color = Color.RED;

            texts.add(fpsText);
            texts.add(usbError);
            texts.add(videoDeviceError);
            texts.add(camStartingError);
            texts.add(videoIoctlError);
            texts.add(servoError);
            updateErrors();

            switch(currentOperation)
            {
                case STARTUP: {
                    String text = "WELCOME TO TRACKER";
                    size = Text.calculateSize(text);
                    if(landscape)
                    {
                        x = canvas.getWidth() - MARGIN;
                        y = canvas.getHeight() / 4 - size.width() / 2;
                    }
                    else
                    {
                        x = canvas.getWidth() / 2 - size.width() / 2;
                        y = canvas.getHeight() / 2 + MARGIN;
                    }

                    if(landscape) {
                        texts.add(new Text(x - size.height() / 2, y, text));
                    }
                    else {
                        texts.add(new Text(x, y + size.height() / 2, text));
                    }


                    text = "ACTIVATE MOTORS";
                    if(landscape) {
                        x = canvas.getWidth() / 2;
                        y = canvas.getHeight() / 4;
                    }
                    else
                    {
                        x = canvas.getWidth() / 2;
                        y = canvas.getHeight() * 3 / 4;
                    }
                    Button b = new Button(x, y, text);
                    b.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "ACTIVATE MOTORS");
                            client.sendCommand(' ');
                        }
                    };
                    buttons.add(b);
                    break;
                }



                case CONFIGURING: {
                    String text = "ROTATE PHONE FOR";
                    size = Text.calculateSize(text);
                    if (landscape) {
                        x = canvas.getWidth() - MARGIN - size.height() / 2;
                        y = MARGIN;
                    } else {
                        x = MARGIN;
                        y = canvas.getHeight() / 2 + MARGIN + size.height() / 2;
                    }
                    Text t = new Text(x, y, text);
                    texts.add(t);
                    if(landscape) {
                        text = "PORTRAIT MODE";
                        x -= size.height() + MARGIN;
                    }
                    else
                    {
                        text = "LANDSCAPE MODE";
                        y += size.height() + MARGIN;
                    }
                    t = new Text(x, y, text);
                    texts.add(t);

                    text = "ABORT";
                    size = Button.calculateSize(text, null);
                    if (landscape) {
                        x = size.height() / 2 + 1;
                        y = canvas.getHeight() / 8;
                    } else {
                        x = size.width() / 2 + MARGIN;
                        y = canvas.getHeight() - size.height() / 2 - MARGIN;
                    }
                    Button stop = new Button(x, y, text);
                    stop.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "STOP");
                            client.sendCommand('q');
                        }
                    };
                    buttons.add(stop);

                    text = "START TRACKING";
                    size = Button.calculateSize(text, null);
                    if (landscape) {
                        y = canvas.getHeight() * 3 / 8;
                    } else {
                        x = canvas.getWidth() - size.width() / 2 - MARGIN;
                    }
                    Button start = new Button(x, y, text);
                    start.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "START");
                            // different code to prevent buffered keypresses
                            client.sendCommand('\n');
                        }
                    };
                    buttons.add(start);

//
//                    x += size.width() * 2;
//
//                    text = landscapeText();
//                    landscapeButton = new Button(x,videoRect.top / 4, text);
//                    landscapeButton.listener = new Button.ButtonListener() {
//                        @Override
//                        public void onClick() {
//                            Log.i("FirstFragment", "ROTATE");
//                            client.sendCommand('r');
//                        }
//                    };
//                    buttons.add(landscapeButton);

                    if (landscape) {
                        y = MARGIN;
                        x += start.getW() + MARGIN;
                    } else {
                        y -= size.height() + MARGIN;
                        x = MARGIN;
                    }

                    t = new Text(x, y, "PAN SIGN:");
                    texts.add(t);

                    // fixed extents for the pan & tilt signs
                    size = Button.calculateSize("-1", null);
                    if (landscape) {
                        y += t.getH() + size.height() / 2 + MARGIN;
                    } else {
                        x += t.getW() + size.width() / 2 + MARGIN;
                    }
                    text = String.valueOf(pan_sign);
                    panButton = new Button(x, y, text);
                    panButton.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "PAN SIGN");
                            client.sendCommand('p');
                        }
                    };
                    buttons.add(panButton);

                    if (landscape) {
                        y = canvas.getHeight() / 4;
                    } else {
                        x = canvas.getWidth() / 2;
                    }
                    t = new Text(x, y, "TILT SIGN:");
                    texts.add(t);

                    if (landscape) {
                        y += t.getH() + size.height() / 2 + MARGIN;
                    } else {
                        x += t.getW() + size.width() / 2 + MARGIN;
                    }
                    text = String.valueOf(tilt_sign);
                    tiltButton = new Button(x, y, text);
                    tiltButton.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "TILT SIGN");
                            client.sendCommand('t');
                        }
                    };

                    buttons.add(tiltButton);


                    size = Text.calculateSize("X");
                    if (landscape) {
                        x += tiltButton.getW() / 2 + size.width() + MARGIN;
                        y = MARGIN;
                    } else {
                        x = MARGIN;
                        y -= tiltButton.getH() / 2 + MARGIN + size.height() / 2;
                    }


                    panText = new Text(x, y, getPanText());
                    texts.add(panText);

                    if (landscape) {
                        y = canvas.getHeight() / 4;
                    } else {
                        x = canvas.getWidth() / 2;
                    }

                    tiltText = new Text(x, y, getTiltText());
                    texts.add(tiltText);

                    text = lensText();
                    size = Button.calculateSize(text, null);
                    if (landscape) {
                        x += tiltText.getW() + MARGIN + size.width() / 2;
                        y = MARGIN + size.height() / 2;
                    } else {
                        x = MARGIN + size.width() / 2;
                        y -= tiltText.getH() / 2 + MARGIN + size.height() / 2;
                    }
                    lensButton = new Button(x, y, text);
                    lensButton.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "LENS");
                            client.sendCommand('l');
                        }
                    };
                    buttons.add(lensButton);

                    text = "CENTER";
                    size = Button.calculateSize(text, null);
                    if (landscape) {
                        x += lensButton.getW() + MARGIN;
                    } else {
                        y -= lensButton.getH() + MARGIN;
                    }
                    Button b = new Button(x, y, text);
                    b.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "CENTER");
                            client.sendCommand('c');
                        }
                    };
                    buttons.add(b);



                    // arrow dimensions
                    int w = canvas.getWidth() / 8;
                    int h = w;
                    int centerX;
                    int centerY;
                    if(landscape) {
                        centerX = canvas.getWidth() - w - ARROW_MARGIN - w / 2 - MARGIN;
                        centerY = canvas.getHeight() / 2 - h - ARROW_MARGIN - w / 2 - MARGIN;
                    }
                    else
                    {
                        centerX = canvas.getWidth()  - w - ARROW_MARGIN - w / 2 - MARGIN;
                        centerY = canvas.getHeight() / 2 + MARGIN + h + ARROW_MARGIN + h / 2;
                    }

                    b = new Button(Button.calculateRect(centerX,
                            centerY - h - ARROW_MARGIN,
                            w,
                            h), Button.LEFT);
                    b.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "LEFT");
                            if(landscape) {
                                client.sendCommand('a');
                            }
                            else
                            {
                                client.sendCommand('w');
                            }
                        }
                    };
                    buttons.add(b);

                    b = new Button(Button.calculateRect(centerX,
                            centerY + h + ARROW_MARGIN,
                            w,
                            h), Button.RIGHT);
                    b.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "RIGHT");
                            if(landscape) {
                                client.sendCommand('d');
                            }
                            else
                            {
                                client.sendCommand('s');
                            }
                        }
                    };
                    buttons.add(b);

                    b = new Button(Button.calculateRect(centerX + w + ARROW_MARGIN,
                            centerY,
                            w,
                            h), Button.UP);
                    b.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "UP");
                            if(landscape) {
                                client.sendCommand('w');
                            }
                            else
                            {
                                client.sendCommand('d');
                            }
                        }
                    };
                    buttons.add(b);


                    b = new Button(Button.calculateRect(centerX - w - ARROW_MARGIN,
                            centerY,
                            w,
                            h), Button.DOWN);
                    b.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "DOWN");
                            if(landscape) {
                                client.sendCommand('s');
                            }
                            else
                            {
                                client.sendCommand('a');
                            }
                        }
                    };
                    buttons.add(b);



                    break;
                }




                case TRACKING: {
                    String text = "ABORT";
                    size = Button.calculateSize(text, null);
                    if(landscape) {

                        x = canvas.getWidth() / 2 + MARGIN / 2 + size.width() / 2;
                        y = MARGIN + size.height() / 2;
                    }
                    else
                    {
                        x = MARGIN + size.width() / 2;
                        y = canvas.getHeight() - MARGIN - size.height() / 2;
                    }

                    Button b = new Button(x,
                            y, text);
                    b.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "STOP2");
                            client.sendCommand('Q');
                        }
                    };
                    buttons.add(b);

                    text = "BACK";
                    size = Button.calculateSize(text, null);
                    if(landscape) {
                        x = canvas.getWidth() / 2 - MARGIN / 2 - size.width() / 2;
                        y = MARGIN + size.height() / 2;
                    }
                    else
                    {
                        x = canvas.getWidth() - MARGIN - size.width() / 2;
                    }
                    b = new Button(x,
                            y, text);
                    b.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "BACK");
                            client.sendCommand('b');
                        }
                    };
                    buttons.add(b);
                    break;
                }
            }

            drawGUI(canvas);
            video.getHolder().unlockCanvasAndPost(canvas);

        }
        else
        {
            currentOperation = OFF;
        }

    }

    String getPanText()
    {
        String center = (start_pan == pan) ? "*" : "";
        return "PAN: " + String.valueOf(pan) + center;
    }

    String getTiltText()
    {
        String center = (start_tilt == tilt) ? "*" : "";
        return "TILT: " + String.valueOf(tilt) + center;
    }

    public void changeOperation() {
        getActivity().runOnUiThread(new Runnable() {
                                        @Override
                                        public void run() {
                                            changeOperationSync();
                                        }
                                    }
        );
    }

    private String lensText() {
        final int LENS_15 = 0;
        final int LENS_28 = 1;
        final int LENS_50 = 2;
        switch(lens)
        {
            case LENS_15:
                return "17MM";
            case LENS_28:
                return "28MM";
            case LENS_50:
                return "50MM";
            default:
                return "UNKNOWN";
        }
    }

    private String landscapeText() {
        if(landscape)
        {
            return "LANDSCAPE";
        }
        else
        {
            return "PORTRAIT";
        }
    }

    public void drawGUI(Canvas canvas) {
        Paint p = new Paint();
        p.setStyle(Paint.Style.FILL);

        // erase background
        p.setColor(Color.DKGRAY);
        canvas.drawRect(new Rect(0, 0, canvas.getWidth(), canvas.getHeight()), p);
//        Rect videoRect = new Rect();

        // blitted size of the video
//        if(landscape) {
//            videoRect.top = canvas.getHeight() / 2;
//            videoRect.bottom = canvas.getHeight();
//            int scaledW = (videoRect.bottom - videoRect.top) * H / W;
//            videoRect.left = canvas.getWidth() / 2 - scaledW / 2;
//            videoRect.right = videoRect.left + scaledW;
//        }
//        else
//        {
//            videoRect.left = 0;
//            videoRect.right = canvas.getWidth();
//            // 3:2 aspect ratio
//            int scaledW = videoRect.width() * 2 / 3;
//            videoRect.top = canvas.getHeight() / 2 - scaledW / 2;
//            videoRect.bottom = videoRect.top + scaledW;
//        }

// position for tracking vs. configuration
        dstX = canvas.getWidth() / 2;
        dstY = canvas.getHeight() / 2;
        if(currentOperation != TRACKING)
        {
            if(landscape)
            {
                dstY = canvas.getHeight() * 3 / 4;
            }
            else
            {
                dstY = canvas.getHeight() / 4;
            }
        }

        Matrix matrix = new Matrix();
        matrix.reset();
        matrix.postTranslate(-W / 2, -H / 2); // Centers source image
        if(!landscape)
        {
            float xScale;
            float yScale;
            //Log.i("FirstFragment", "drawGUI canvas.getHeight()=" + canvas.getHeight() + " dstH=" + dstH + " cropped_w=" + cropped_w);

            if(currentOperation != TRACKING)
            {
                dstH = canvas.getHeight() / 2;
                dstW = dstH * 9 / 16;
            }
            else
            {
                dstW = canvas.getWidth();
                dstH = dstW * 16 / 9;
            }
            xScale = (float) dstH / W;
            yScale = (float) dstW / H;
            matrix.postScale(xScale, yScale);
            matrix.postRotate(-90);
            matrix.postTranslate(dstX, dstY);
        }
        else
        {
            float scale1;
            float scale2;
            float scale;
            if(currentOperation != TRACKING)
            {
                dstW = canvas.getWidth() / 2;
                dstH = canvas.getHeight() / 2;
                if(dstH / dstW > 16.0 / 9)
                    dstW = dstH * 9 / 16;
                scale = dstH / W;
            }
            else 
            {
                dstW = canvas.getWidth();
                dstH = canvas.getHeight();
                if(dstH / dstW > 16.0 / 9)
                    dstH = dstW * 16 / 9;
                scale = dstW / H;
            }
            scale1 = (float) dstH / W;
            scale2 = (float) dstW / H;
            matrix.postScale(scale, scale);
            matrix.postRotate(90);
            matrix.postTranslate(dstX, dstY);
        }


        canvas.drawBitmap(videoBitmap,
                matrix,
                p);

//Log.i("drawGUI", "dstW=" + dstW + " dstH=" + dstH);
// draw keypoints
        for(int j = 0; j < animals; j++)
        {
            for(int i = 0; i < colors.length; i++)
            {
                p.setColor(colors[i]);
                drawBodyPart(j, 
                    i, 
                    canvas, 
                    p);
            }
            joinBodyParts(canvas, p, j, MODEL_SHOULDER1, MODEL_NECK);
            joinBodyParts(canvas, p, j, MODEL_SHOULDER2, MODEL_NECK);
            joinBodyParts(canvas, p, j, MODEL_ELBOW2, MODEL_SHOULDER2);
            joinBodyParts(canvas, p, j, MODEL_ELBOW1, MODEL_SHOULDER1);
            joinBodyParts(canvas, p, j, MODEL_WRIST2, MODEL_ELBOW2);
            joinBodyParts(canvas, p, j, MODEL_WRIST1, MODEL_ELBOW1);
            joinBodyParts(canvas, p, j, MODEL_NECK, MODEL_BUTT);
            joinBodyParts(canvas, p, j, MODEL_HIP1, MODEL_BUTT);
            joinBodyParts(canvas, p, j, MODEL_HIP2, MODEL_BUTT);
            joinBodyParts(canvas, p, j, MODEL_KNEE1, MODEL_HIP1);
            joinBodyParts(canvas, p, j, MODEL_KNEE2, MODEL_HIP2);
            joinBodyParts(canvas, p, j, MODEL_ANKLE1, MODEL_KNEE1);
            joinBodyParts(canvas, p, j, MODEL_ANKLE2, MODEL_KNEE2);
            joinBodyParts(canvas, p, j, MODEL_NOSE, MODEL_NECK);
            joinBodyParts(canvas, p, j, MODEL_EYE2, MODEL_NOSE);
            joinBodyParts(canvas, p, j, MODEL_EYE1, MODEL_NOSE);
            joinBodyParts(canvas, p, j, MODEL_EAR1, MODEL_EYE1);
            joinBodyParts(canvas, p, j, MODEL_EAR2, MODEL_EYE2);
            joinBodyParts(canvas, p, j, MODEL_HEEL1, MODEL_ANKLE1);
            joinBodyParts(canvas, p, j, MODEL_HEEL2, MODEL_ANKLE2);
            joinBodyParts(canvas, p, j, MODEL_BIGTOE1, MODEL_ANKLE1);
            joinBodyParts(canvas, p, j, MODEL_BIGTOE2, MODEL_ANKLE2);
            joinBodyParts(canvas, p, j, MODEL_SMALLTOE1, MODEL_BIGTOE1);
            joinBodyParts(canvas, p, j, MODEL_SMALLTOE2, MODEL_BIGTOE2);
        }

// update FPS
        StringBuilder sb = new StringBuilder();
        Formatter formatter = new Formatter(sb);
        
        if(fpsText != null) fpsText.updateText(formatter.format("%.02f", fps).toString());

        for (int i = 0; i < buttons.size(); i++)
        {
            buttons.get(i).draw(canvas);
        }

        for (int i = 0; i < texts.size(); i++)
        {
            texts.get(i).draw(canvas);
        }



    }



    static public void drawBodyPart(int animal,
        int bodyPart, 
        Canvas c, 
        Paint p)
    {
        int radius;
        if(currentOperation != TRACKING)
            radius = 4;
        else
            radius = 8;
        int offset = animal * BODY_PARTS * 2 + bodyPart * 2;
        int x = keypoints[offset];
        int y = keypoints[offset + 1];
        if(x > 0 || y > 0) 
        {
            int x2 = 0;
            int y2 = 0;
// scale to screen positions
            if(landscape)
            {
                x2 = (int)(dstX + dstW / 2 - y * dstW / H);
                y2 = (int)(dstY - dstH / 2 + x * dstH / W);
            }
            else
            {
                x2 = (int)(dstX - dstW / 2 + x * dstW / H);
// crop & scale portrait height
                float portraitScale = (float)(540.0 / 640);
                y2 = (int)(dstY - dstH * portraitScale / 2 + y * dstH / W);
            }

            p.setStrokeWidth(1);
            p.setStyle(Paint.Style.FILL);
            c.drawCircle(x2, y2, radius, p);
            keypoint_cache[offset] = x2;
            keypoint_cache[offset + 1] = y2;
        }
        else
        {
            keypoint_cache[offset] = 0;
            keypoint_cache[offset + 1] = 0;
        }
    }

    static public void joinBodyParts(Canvas c, 
        Paint p,
        int animal,
        int bodyPart1, 
        int bodyPart2)
    {
        int offset = animal * BODY_PARTS * 2;
        int x1 = keypoint_cache[offset + bodyPart1 * 2];
        int y1 = keypoint_cache[offset + bodyPart1 * 2 + 1];
        int x2 = keypoint_cache[offset + bodyPart2 * 2];
        int y2 = keypoint_cache[offset + bodyPart2 * 2 + 1];

        if((x1 > 0 || y1 > 0) && (x2 > 0 || y2 > 0))
        {
            p.setStyle(Paint.Style.STROKE);
            p.setColor(colors[bodyPart1]);
            p.setStrokeWidth(4);
            c.drawLine(x1, y1, x2, y2, p);
        }
    }


    @Override
    public boolean onTouch(View view, MotionEvent motionEvent) {
        int pointers = motionEvent.getPointerCount();
        Log.i("FirstFragment", "motionEvent.getAction()=" + motionEvent.getAction());

        boolean needRedraw = false;
        for(int i = 0; i < buttons.size(); i++)
        {
            if(buttons.get(i).onTouch(motionEvent))
            {
                needRedraw = true;
                break;
            }
        }

        if(needRedraw)
        {
            Canvas canvas = video.getHolder().lockCanvas();
            drawGUI(canvas);
            video.getHolder().unlockCanvasAndPost(canvas);
        }

        return true;
    }


    @Override
    public void onSensorChanged(SensorEvent e) {
        float x = e.values[0];
        float y = e.values[1];
        accelX = accelX * ((float)1.0 - ACCEL_BANDWIDTH) + x * ACCEL_BANDWIDTH;
        accelY = accelY * ((float)1.0 - ACCEL_BANDWIDTH) + y * ACCEL_BANDWIDTH;
//        Log.i("onSensorChanged", "x=" + accelX + " y=" + accelY);

        if(currentOperation == CONFIGURING) {
            if (accelX > 5 && accelY < 1) {
                if (!landscape) {
                    client.sendCommand('r');
                }
            } else if (accelY > 5 && accelX < 1) {
                if (landscape) {
                    client.sendCommand('R');
                }
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {
    }

}
