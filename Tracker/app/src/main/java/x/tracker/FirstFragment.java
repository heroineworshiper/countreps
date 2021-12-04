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

import com.arthenica.ffmpegkit.ExecuteCallback;
import com.arthenica.ffmpegkit.FFmpegKitConfig;
import com.arthenica.ffmpegkit.FFmpegSession;
import com.arthenica.ffmpegkit.Session;
import com.arthenica.ffmpegkit.FFmpegKit;

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
import java.util.Vector;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.navigation.fragment.NavHostFragment;
import x.tracker.databinding.FragmentFirstBinding;

import static android.content.Context.MODE_APPEND;

public class FirstFragment extends Fragment implements View.OnTouchListener {

    private FragmentFirstBinding binding;
    SurfaceView video;
    ByteBuffer[] frameBuffer = new ByteBuffer[2];
    int currentFrameBuffer = 0;
    Bitmap videoBitmap;
    Rect videoRect = new Rect();
    ClientThread client;


    // size of the encoded video
    static final int W = 640;
    static final int H = 360;
    static OutputStream ffmpeg_stdin;
    static InputStream ffmpeg_stdout;
    static String stdinPath;
    static String stdoutPath;



    final int OFF = -1;
    final int STARTUP  = 0;
    final int CONFIGURING = 1;
    final int TRACKING = 2;
    int currentOperation = OFF;
    int prevOperation = OFF;

    int pan;
    int tilt;
    int pan_sign;
    int tilt_sign;
    int lens;
    int landscape;

// error codes
    final int VIDEO_DEVICE_ERROR = 1;
    final int VIDEO_BUFFER_ERROR = 2;
    final int SERVO_ERROR = 4;

    int errors;

    Vector<Button> buttons = new Vector();
    Vector<Text> texts = new Vector();
    Button landscapeButton;
    Button lensButton;
    Button tiltButton;
    Button panButton;
    Text panText;
    Text tiltText;
    Text videoDeviceError;
    Text videoBufferError;
    Text servoError;
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


        videoBitmap = Bitmap.createBitmap(W, H, Bitmap.Config.ARGB_8888);

        for(int i = 0; i < 2; i++) {
            frameBuffer[i] = ByteBuffer.allocateDirect(videoBitmap.getByteCount());
        }

        stdinPath = FFmpegKitConfig.registerNewFFmpegPipe(getContext());
        stdoutPath = FFmpegKitConfig.registerNewFFmpegPipe(getContext());
        Log.i("FirstFragment", "onViewCreated " + stdinPath + " " + stdoutPath);

        new Thread(new DecodeThread(this)).start();
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

                            // compute extents which depend on the canvas
                            videoRect.top = canvas.getHeight() / 2;
                            videoRect.bottom = canvas.getHeight();
                            int scaledW = (videoRect.bottom - videoRect.top) * H / W;
                            videoRect.left = canvas.getWidth() / 2 - scaledW / 2;
                            videoRect.right = videoRect.left + scaledW;



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
                    p.setColor(Color.WHITE);
                    p.setStyle(Paint.Style.FILL);
                    p.setTypeface(Typeface.create("SansSerif", Typeface.BOLD));
                    p.setTextSize(70);
                    Rect text_size = new Rect();
                    p.getTextBounds(text,
                               0,
                               text.length(),
                               text_size);
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

    public void drawVideo() {
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Canvas canvas = video.getHolder().lockCanvas();

                if(canvas != null) {
//                    Paint p = new Paint();
//                    p.setColor(Color.BLACK);
//                    p.setStyle(Paint.Style.FILL);
//
//                    canvas.drawRect(new Rect(0, 0, canvas.getWidth(), canvas.getHeight()), p);

                    int current = currentFrameBuffer - 1;
                    if (current < 0)
                    {
                        current = 1;
                    }
                    videoBitmap.copyPixelsFromBuffer(frameBuffer[current]);
                    frameBuffer[current].rewind();
                    drawGUI(canvas);

                    video.getHolder().unlockCanvasAndPost(canvas);
                }
            }
        });
    }

    boolean updateErrors()
    {
        boolean needRedraw = false;
        needRedraw |= videoDeviceError.setHidden((errors & VIDEO_DEVICE_ERROR) == 0);
        needRedraw |= videoBufferError.setHidden((errors & VIDEO_BUFFER_ERROR) == 0);
        needRedraw |= servoError.setHidden((errors & SERVO_ERROR) == 0);
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
                    needRedraw |= panText.updateText("PAN: " + String.valueOf(pan));
                }
                if(tiltText != null)
                {
                    needRedraw |= tiltText.updateText("TILT: " + String.valueOf(tilt));
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


    public void changeOperation() {
        getActivity().runOnUiThread(new Runnable() {
                                        @Override
                                        public void run() {
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
                                                int x = canvas.getWidth() / 4;
                                                int y = canvas.getHeight() / 2;

                                                videoDeviceError = new Text(x, y, "VIDEO DEVICE NOT FOUND");
                                                videoDeviceError.color = Color.RED;
                                                x += canvas.getWidth() / 4;
                                                videoBufferError = new Text(x, y, "VIDEO CAPTURE FAILED");
                                                videoBufferError.color = Color.RED;
                                                x += canvas.getWidth() / 4;
                                                servoError = new Text(x, y, "SERVO DEVICE NOT FOUND");
                                                servoError.color = Color.RED;

                                                texts.add(videoDeviceError);
                                                texts.add(videoBufferError);
                                                texts.add(servoError);
                                                updateErrors();

                                                switch(currentOperation)
                                                {
                                                    case STARTUP: {
                                                        String text = "ACTIVATE MOTORS";

                                                        Button b = new Button(
                                                                canvas.getWidth() / 2,
                                                                videoRect.top / 2,
                                                                text);
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
                                                        String text = "X";
                                                        Rect buttonRect = Button.calculateRect(text,
                                                                canvas.getWidth() / 2,
                                                                canvas.getHeight() / 2);
                                                        x = buttonRect.width() / 2 + 1;
                                                        y = videoRect.top * 1 / 4;
                                                        text = "ABORT";
                                                        Button stop = new Button(x, y, text);
                                                        stop.listener = new Button.ButtonListener() {
                                                            @Override
                                                            public void onClick() {
                                                                Log.i("FirstFragment", "STOP");
                                                                client.sendCommand('q');
                                                            }
                                                        };
                                                        buttons.add(stop);

                                                        y = videoRect.top * 3 / 4;
                                                        text = "START TRACKING";
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


                                                        x += buttonRect.width() * 2;

                                                        text = landscapeText();
                                                        landscapeButton = new Button(x,videoRect.top / 4, text);
                                                        landscapeButton.listener = new Button.ButtonListener() {
                                                            @Override
                                                            public void onClick() {
                                                                Log.i("FirstFragment", "ROTATE");
                                                                client.sendCommand('r');
                                                            }
                                                        };
                                                        buttons.add(landscapeButton);

                                                        text = lensText();
                                                        lensButton = new Button(x,videoRect.top * 3 / 4, text);
                                                        lensButton.listener = new Button.ButtonListener() {
                                                            @Override
                                                            public void onClick() {
                                                                Log.i("FirstFragment", "LENS");
                                                                client.sendCommand('l');
                                                            }
                                                        };
                                                        buttons.add(lensButton);

                                                        y = 0;
                                                        x += lensButton.getW() + MARGIN;
                                                        Text t = new Text(x, 0, "PAN SIGN:");
                                                        texts.add(t);

                                                        // fixed extents for the pan & tilt signs
                                                        buttonRect = Button.calculateRect("-1",
                                                                canvas.getWidth() / 2,
                                                                canvas.getHeight() / 2);
                                                        y = t.getH() + buttonRect.height() / 2 + MARGIN;
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

                                                        y = videoRect.top / 2;
                                                        t = new Text(x, y, "TILT SIGN:");
                                                        texts.add(t);


                                                        y += t.getH() + buttonRect.height() / 2 + MARGIN;
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
                                                        x += tiltButton.getW();



                                                        panText = new Text(x, 0, "PAN: " + String.valueOf(pan));
                                                        texts.add(panText);
                                                        x += panText.rect.width() + MARGIN;
                                                        tiltText = new Text(x, 0, "TILT: " + String.valueOf(tilt));
                                                        texts.add(tiltText);


                                                        // arrow dimensions
                                                        int w = canvas.getWidth() / 8;
                                                        int h = w;
                                                        int centerX = canvas.getWidth() - w - ARROW_MARGIN - w / 2;
                                                        int centerY = canvas.getHeight() * 2 / 8;
                                                        Button b = new Button(Button.calculateRect(centerX,
                                                                centerY - h - ARROW_MARGIN,
                                                                w,
                                                                h), Button.LEFT);
                                                        b.listener = new Button.ButtonListener() {
                                                            @Override
                                                            public void onClick() {
                                                                Log.i("FirstFragment", "LEFT");
                                                                client.sendCommand('a');
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
                                                                client.sendCommand('d');
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
                                                                client.sendCommand('w');
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
                                                                client.sendCommand('s');
                                                            }
                                                        };
                                                        buttons.add(b);



                                                        break;
                                                    }




                                                    case TRACKING: {
                                                        String text = "ABORT";
                                                        Rect buttonRect = Button.calculateRect(text,
                                                                canvas.getWidth() / 2,
                                                                canvas.getHeight() / 2);
                                                        x = buttonRect.width() / 2 + 1;
                                                        y = buttonRect.height() / 2;

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

                                                        y += b.getH() + MARGIN;
                                                        text = "BACK";
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
                return "15-17MM";
            case LENS_28:
                return "28MM";
            case LENS_50:
                return "50MM";
            default:
                return "UNKNOWN";
        }
    }

    private String landscapeText() {
        if(landscape == 1)
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

        p.setColor(Color.DKGRAY);
        canvas.drawRect(new Rect(0, 0, canvas.getWidth(), canvas.getHeight()), p);

        // blitted size of the video
        int dstH = canvas.getHeight() / 2;
        int dstW = canvas.getWidth() / 2;
        int dstX = canvas.getWidth() / 2;
        int dstY = canvas.getHeight() * 3 / 4;

        // full screen when tracking
        if(currentOperation == TRACKING)
        {
            dstY = canvas.getHeight() / 2;
            if(landscape == 1)
            {
                dstW = canvas.getWidth();
            }
        }

        Matrix matrix = new Matrix();
        matrix.reset();
        matrix.postTranslate(-W / 2, -H / 2); // Centers image
        if(landscape == 0)
        {
            float cropped_w = (float)H * 3 / 2;
            float xScale = (float) dstH / W + (float).02;
            float yScale = (float) dstH / cropped_w + (float).02;
            Log.i("FirstFragment", "drawGUI canvas.getHeight()=" + canvas.getHeight() + " dstH=" + dstH + " cropped_w=" + cropped_w);

            matrix.postScale(xScale, -yScale);
            matrix.postRotate(180);
        }
        else
        {
            float scale = (float) dstW / H + (float).09;
            matrix.postScale(scale, scale);
            matrix.postRotate(90);
        }
        matrix.postTranslate(dstX, dstY);


        canvas.drawBitmap(videoBitmap,
                matrix,
                p);

        for (int i = 0; i < buttons.size(); i++)
        {
            buttons.get(i).draw(canvas);
        }

        for (int i = 0; i < texts.size(); i++)
        {
            texts.get(i).draw(canvas);
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

}
