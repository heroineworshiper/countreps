package x.tracker;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Rect;
import android.graphics.Typeface;
import android.util.Log;
import android.view.MotionEvent;

public class Button {
    int centerX;
    int centerY;
    Rect rect;
    String text = "";

    static final int NO_ARROW = 0;
    static final int UP = 1;
    static final int DOWN = 2;
    static final int LEFT = 3;
    static final int RIGHT = 4;
    int arrow = NO_ARROW;

    static final int MARGIN = 20;

    boolean isDown = false;
    boolean isHighlighted = false;
    ButtonListener listener = null;
    Thread repeater;


    interface ButtonListener
    {
        void onClick();
    }


//    Button(Rect rect, String text)
//    {
//        this.rect = rect;
//        this.text = text;
//    }

    Button(Rect rect, int arrow)
    {
        this.rect = rect;
        this.arrow = arrow;
    }

    Button(int centerX, int centerY, String text)
    {
        this.centerX = centerX;
        this.centerY = centerY;
        this.rect = calculateRect(text, centerX, centerY);
        this.text = text;
    }

    // calculate arrow extents
    static Rect calculateRect(int centerX, int centerY, int w, int h) {
        Rect result = new Rect();
        result.left = centerX - w / 2;
        result.top = centerY - h / 2;
        result.right = result.left + w;
        result.bottom = result.top + h;
        return result;
    }

    static Rect calculateRect(String text, int centerX, int centerY)
    {
        Rect result = new Rect();
        Rect textRect = calculateSize(text);

        int textW = textRect.right - textRect.left;
        int textH = textRect.bottom - textRect.top;
        result.top = centerY - textW / 2 - MARGIN;
        result.bottom = result.top + textW + MARGIN * 2;
        result.left = centerX - textH / 2 - MARGIN;
        result.right = result.left + textH + MARGIN * 2;
//        Log.i("Button", "textSize=" + textRect.top + " " + textRect.bottom + " " + textRect.left + " " + textRect.right);
//        Log.i("Button", "center=" + centerX + " " + centerY);
//        Log.i("Button", "rect=" + result.top + " " + result.bottom + " " + result.left + " " + result.right);

        return result;
    }

    static Rect calculateSize(String text)
    {
        Paint p = new Paint();
        Rect text_size = new Rect();
        p.setStyle(Paint.Style.FILL);
        p.setTypeface(Typeface.create("SansSerif", Typeface.BOLD));
        p.setTextSize(FirstFragment.TEXT_SIZE);
        p.getTextBounds(text,
                0,
                text.length(),
                text_size);
        return text_size;
    }

    public boolean updateText(String text) {
        if(!this.text.equals(text)) {
            this.text = text;
            this.rect = calculateRect(text, centerX, centerY);
            return true;
        }
        return false;
    }

    int getW()
    {
        return rect.width();
    }

    int getH()
    {
        return rect.height();
    }

    public void draw(Canvas canvas)
    {
//        Log.i("Button", "draw");
        Paint p = new Paint();

        p.setColor(Color.GREEN);

        // invert it if down
        if(isDown)
        {
            p.setStyle(Paint.Style.FILL);
        }
        else
        {
            p.setStyle(Paint.Style.STROKE);
        }
        p.setStrokeWidth(2);

        if(arrow != NO_ARROW) {
            final double[][] shape =
            {
                { -.5,  -.5 },
                { 0.0,  .5 },
                { .5,  -.5 },
                { -.5,  -.5 },
            };

            Path path = new Path();
            // Rotate shape & scale
            double angle = 0;
            double length = rect.height();
            double width = rect.width();
            switch(arrow)
            {
                case UP:
                    angle = -Math.PI / 2;
                    break;
                case DOWN:
                    angle = Math.PI / 2;
                    break;
                case LEFT:
                    angle = -Math.PI;
                    break;
                case RIGHT:
                    angle = 0;
                    break;
            }
            for(int i = 0; i < shape.length; i++)
            {

                double[] point = shape[i];
                double x1 = point[0] * length;
                double y1 = point[1] * width;
                double r = Math.sqrt(x1 * x1 + y1 * y1);
                double a = Math.atan2(y1, x1);
                a += angle;
                double x = (rect.left + rect.right) / 2;
                double y = (rect.top + rect.bottom) / 2;
                x1 = (float)(x + r * Math.cos(a));
                y1 = (float)(y + r * Math.sin(a));

                if(i == 0)
                    path.moveTo((float)x1, (float)y1);
                else
                    path.lineTo((float)x1, (float)y1);
            }

            canvas.drawPath(path, p);
        }
        else
        {

            canvas.drawRect(rect, p);

            if (isDown) {
                p.setColor(Color.BLACK);
            }

            if (!text.isEmpty()) {
                p.setStyle(Paint.Style.FILL);
                p.setTypeface(Typeface.create("SansSerif", Typeface.BOLD));
                p.setTextSize(FirstFragment.TEXT_SIZE);
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
                //matrix.postTranslate(-temp.getWidth() / 2,
                //        -temp.getHeight() / 2); // Centers image
                matrix.postRotate(90);
                matrix.postTranslate(temp.getHeight() / 2 + (rect.left + rect.right) / 2,
                        -temp.getWidth() / 2 + (rect.top + rect.bottom) / 2);
                canvas.drawBitmap(temp2,
                        matrix,
                        p);

            }
        }
    }

    public boolean onTouch(MotionEvent motionEvent) {
        switch(motionEvent.getAction())
        {
            case MotionEvent.ACTION_DOWN:
                if(motionEvent.getX() >= rect.left &&
                    motionEvent.getX() < rect.right &&
                    motionEvent.getY() >= rect.top &&
                    motionEvent.getY() < rect.bottom)
                {
                    isHighlighted = true;
                    isDown = true;
                    if(arrow != NO_ARROW)
                    {
                        if(listener != null)
                        {
                            listener.onClick();
                        }
                        startRepeater();
                    }
                    return true;
                }
                break;

            case MotionEvent.ACTION_MOVE:
                if(isHighlighted) {
                    // always active if it's an arrow
                    if(arrow == NO_ARROW) {
                        boolean newDown = false;

                        if (motionEvent.getX() >= rect.left &&
                                motionEvent.getX() < rect.right &&
                                motionEvent.getY() >= rect.top &&
                                motionEvent.getY() < rect.bottom) {
                            newDown = true;
                        }

                        if (newDown != isDown) {
                            isDown = newDown;
                            return true;
                        }
                    }
                }
                break;

            case MotionEvent.ACTION_UP:
                if(isHighlighted)
                {
                    isHighlighted = false;
                    if(isDown)
                    {
                        if(listener != null && arrow == NO_ARROW)
                        {
                            listener.onClick();
                        }
                        isDown = false;
                    }
                    if(arrow != NO_ARROW)
                    {
                        stopRepeater();
                    }
                    return true;
                }
                break;
            default:
                //Log.i("Button", "onTouch id=" + motionEvent.getAction());
                break;
        }

        return false;
    }

    private void startRepeater() {
        repeater = new Thread()
        {
            public void run()
            {
                int repeats = 0;
                boolean interrupted = false;
                while(!interrupted)
                {
                    try {
                        if(repeats == 0)
                        {
                            Thread.sleep(1000L);
                        }
                        else
                        {
                            Thread.sleep(150L);
                        }
                    } catch (InterruptedException e) {
                        interrupted = true;
                        e.printStackTrace();
                    }

                    if(!interrupted) {
                        repeats++;
                        if(listener != null)
                        {
                            listener.onClick();
                        }

                    }
                }
            }
        };
        repeater.start();
    }

    void stopRepeater()
    {
        repeater.interrupt();
        try {
            repeater.join();
        } catch (Exception e) {
            e.printStackTrace();
        }
        repeater = null;
    }
}
