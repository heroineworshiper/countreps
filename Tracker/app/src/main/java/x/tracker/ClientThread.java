package x.tracker;

import android.graphics.Bitmap;
import android.graphics.ImageDecoder;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.Formatter;
import java.util.StringTokenizer;
import java.util.Vector;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

class ClientThread implements Runnable {
    private FirstFragment fragment;

    static String SERVER = "10.0.0.20";
    static final int SEND_PORT = 1234;
    static final int RECV_PORT = 1235;
    static final String SETTINGS = "/sdcard/tracker/settings.txt";

    static Socket socket;
    byte[] header = new byte[8];
    byte[] packet = new byte[65536];

    final int GET_START_CODE0 = 0;
    final int GET_START_CODE1 = 1;
    final int GET_HEADER = 2;
    final int GET_DATA = 3;
    int packetState = GET_START_CODE0;
    int counter = 0;
    int dataSize = 0;

    final int START_CODE0 = 0xff;
    final int START_CODE1 = 0xe7;

    // packet type
    final int VIJEO = 0x00;
    final int STATUS = 0x01;
    ExecutorService pool = Executors.newFixedThreadPool(1);

    ClientThread(FirstFragment fragment)
    {
        this.fragment = fragment;
    }


    public static int write_int32(byte[] data, int offset, int value) {
        data[offset++] = (byte)(value & 0xff);
        data[offset++] = (byte)((value >> 8) & 0xff);
        data[offset++] = (byte)((value >> 16) & 0xff);
        data[offset++] = (byte)((value >> 24) & 0xff);
        return offset;
    }

    static public int read_int32(byte[] data, int offset)
    {
        return (data[offset] & 0xff) |
                ((data[offset + 1] & 0xff) << 8) |
                ((data[offset + 2] & 0xff) << 16) |
                ((data[offset + 3]) << 24);
    }

    static public int read_uint16(byte[] data, int offset)
    {
        return (data[offset] & 0xff) |
                ((data[offset + 1] & 0xff) << 8);
    }


    static public void printBuffer(String string, byte[] buffer, int offset, int bytes)
    {
        StringBuilder sb = new StringBuilder();
        Formatter formatter = new Formatter(sb);
        for(int i = 0; i < bytes; i++)
        {
            formatter.format("%02x ", buffer[i + offset]);
        }
        Log.i(string, sb.toString());
    }

    private void handleStatus() {
        if(fragment.currentOperation != fragment.prevOperation ||
            fragment.prevLandscape != fragment.landscape)
        {
            fragment.changeOperation();
        }
        else
        {
            fragment.updateValues();
        }
    }

    void sendCommand(int id) {
        pool.execute(new Runnable() {
            @Override
            public void run() {
                synchronized (this) {
                    try {
                        DatagramSocket socket = new DatagramSocket();
                        InetAddress address = InetAddress.getByName(SERVER);
                        byte[] buffer = new byte[1];
                        buffer[0] = (byte) id;
                        DatagramPacket packet = new DatagramPacket(
                            buffer,
                            buffer.length,
                            address,
                            SEND_PORT);
                        socket.send(packet);
                        socket.close();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
        });
    }


    public void readSettings()
    {
        File file = new File(SETTINGS);
        BufferedReader reader = null;

        try {
            reader = new BufferedReader(new FileReader(file));
            String line = null;
            while ((line = reader.readLine()) != null)
            {
//				Log.v("Settings", "readSettings " + line);

                StringTokenizer st = new StringTokenizer(line);
                Vector<String> strings = new Vector<String>();
                while(st.hasMoreTokens())
                {
                    strings.add(st.nextToken());
                }

                // comment or whitespace
                if(strings.size() < 2 || strings.get(0).charAt(0) == '#') continue;

                String key = strings.get(0);
                String value = strings.get(1);
                if(key.equalsIgnoreCase("ADDRESS"))
                {
                    SERVER = value;
                }
            }
            reader.close();

        } catch (Exception e) {
            Log.i("ClientThread", "readSettings 1 " + e.toString());
            return;
        }
    }

    @Override
    public void run() {
        DatagramSocket socket = null;
        try {
            socket = new DatagramSocket(RECV_PORT);
            socket.setSoTimeout(1000);
        } catch (SocketException e) {
            e.printStackTrace();
        }
        fragment.waitForSurface();

        readSettings();
// ping the server
        Log.i("ClientThread", "server=" + SERVER + ":" + RECV_PORT);
        boolean gotStatus = false;
        sendCommand('*');

        while (true) {
            // read stream from server
            byte[] buffer = new byte[0x100000];
            DatagramPacket packet_ = new DatagramPacket(buffer, buffer.length);
            boolean timeout = false;
            int bytes_read = 0;
            try {
                socket.receive(packet_);
            } catch (IOException e) {
//                e.printStackTrace();
                timeout = true;
            }

            if(timeout)
            {
                Log.i("ClientThread", "server timed out");
                try {
                    socket.close();
                    socket = new DatagramSocket(RECV_PORT);
                    socket.setSoTimeout(1000);
                } catch (SocketException e) {
                    e.printStackTrace();
                }
                sendCommand('*');
                bytes_read = 0;
            }
            else
            {
                bytes_read = packet_.getLength();
            }

            for(int i = 0; i < bytes_read; i++)
            {
                int c = (int)buffer[i];
                switch(packetState)
                {
                    case GET_START_CODE0:
                        if(c == (byte)START_CODE0)
                        {
                            //Log.i("ClientThread", "START_CODE0");
                            packetState = GET_START_CODE1;
                        }
                        break;
                    case GET_START_CODE1:

                        if(c == (byte)START_CODE1)
                        {
                            //Log.i("ClientThread", "GET_START_CODE1");

                            packetState = GET_HEADER;
                            counter = 2;
                        }
                        else
                        if(c == (byte)START_CODE0)
                        {
                            packetState = GET_START_CODE1;
                        }
                        else
                        {
                            packetState = GET_START_CODE0;
                        }
                        break;
                    case GET_HEADER:
                        header[counter++] = (byte)c;
                        if(counter >= 8)
                        {
                            //Log.i("ClientThread", "GET_HEADER ");
                            dataSize = read_int32(header, 4);
                            packetState = GET_DATA;
                            counter = 0;
                        }
                        break;
                    case GET_DATA:
                        packet[counter++] = (byte)c;
                        if(counter >= dataSize)
                        {
                            int type = (int)header[2];

                            //Log.i("ClientThread", "GET_DATA type=" + type);
                            if(type == VIJEO)
                            {
                                Log.i("ClientThread", "VIJEO size=" + dataSize);

                                if(!FirstFragment.busy)
                                {
                                    FirstFragment.busy = true;
    // extract keypoints
                                    int offset = 0;
                                    int fpsTemp = read_uint16(packet, offset);
                                    offset += 2;
                                    FirstFragment.fps = (float)((int)fpsTemp / 256 + (float)(fpsTemp & 0xff) / 256);
                                    FirstFragment.animals = read_uint16(packet, offset);
    //                                 Log.i("ClientThread", 
    //                                     "KEYPOINTS size=" + dataSize +
    //                                     " animals=" + FirstFragment.animals);
                                    if(2 + FirstFragment.animals * FirstFragment.BODY_PARTS * 4 <= dataSize)
                                    {
                                        offset += 2;
                                        for(int j = 0; j < FirstFragment.animals * FirstFragment.BODY_PARTS * 2; j++)
                                        {
                                            FirstFragment.keypoints[j] = read_uint16(packet, offset);
                                            offset += 2;
                                        }
                                    }

    // extract image
                                    ImageDecoder.Source imageSource =
                                        ImageDecoder.createSource(
                                            ByteBuffer.wrap(packet, offset, dataSize - offset));
                                    // generates a hardware bitmap
                                    Bitmap bitmap = null;
                                    try {
                                        bitmap = ImageDecoder.decodeBitmap(imageSource);
                                    } catch (IOException e) {
                                        e.printStackTrace();
                                    }

    //                                Log.i("ClientThread", "bitmap=" + bitmap);
    //                                                    Log.i("x", "VIJEO w=" + bitmap.getWidth() +
    //                                                            " h=" + bitmap.getHeight() +
    //                                                            " " + bitmap.getColorSpace());

                                    if(bitmap != null)
                                        fragment.drawVideo(bitmap);
                                }
                            }
                            else if(type == STATUS)
                            {
                                fragment.prevOperation = fragment.currentOperation;
                                fragment.prevLandscape = fragment.landscape;

                                fragment.currentOperation = packet[0];
                                fragment.pan = read_int32(packet, 2);
                                fragment.tilt = read_int32(packet, 6);
                                fragment.start_pan = read_int32(packet, 10);
                                fragment.start_tilt = read_int32(packet, 14);
                                fragment.pan_sign = packet[18];
                                fragment.tilt_sign = packet[19];
                                fragment.lens = packet[20];
                                fragment.landscape = (packet[21] == 1 ? true : false);
                                fragment.errors = packet[22] & 0xff;

                                Log.i("ClientThread", "STATUS" +
                                        " currentOperation=" + fragment.currentOperation +
                                        " pan=" + fragment.pan +
                                        " tilt=" + fragment.tilt +
                                        " pan_sign=" + fragment.pan_sign +
                                        " tilt_sign=" + fragment.tilt_sign +
                                        " lens=" + fragment.lens +
                                        " landscape=" + fragment.landscape +
                                        " errors=" + fragment.errors);

                                handleStatus();
                                gotStatus = true;
                            }


                            packetState = GET_START_CODE0;

                            if(!gotStatus)
                            {
// ping the server to get a 1st status packet
                                sendCommand('*');
                            }
                        }
                        break;
                } // switch
            } // bytes_read
        } // while
    } // run
}
