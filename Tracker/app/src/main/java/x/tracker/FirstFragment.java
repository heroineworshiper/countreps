package x.tracker;

import android.app.ActionBar;
import android.content.pm.ActivityInfo;
import android.media.MediaPlayer;
import android.net.Uri;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.VideoView;

import java.io.IOException;
import java.net.InetAddress;
import java.net.Socket;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.navigation.fragment.NavHostFragment;
import x.tracker.databinding.FragmentFirstBinding;

public class FirstFragment extends Fragment {

    private FragmentFirstBinding binding;
    VideoView video;
    static final String SERVER = "10.0.0.20";
    static final int PORT = 1234;

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
        video = binding.videoView;


        video.setVideoURI(Uri.parse("rtsp://10.0.0.20:7654/mystream.sdp"));
        //video.setVideoURI(Uri.parse("http://10.0.0.20:1234/vijeo"));
        //video.setVideoURI(Uri.parse("http://10.0.0.25/2001.mp4"));
        video.setOnErrorListener((MediaPlayer.OnErrorListener) (MediaPlayer mp, int what, int extra) -> {
            Log.i("FirstFragment", "Failed to load vijeo");
         return true;
        });
        video.start();




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
    }

@Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }

        static Socket socket;

        class ClientThread implements Runnable {
            @Override
            public void run() {
                // connect to the server
                try {
                    InetAddress serverAddr = InetAddress.getByName(SERVER);
                    socket = new Socket(serverAddr, PORT);
                } catch (IOException e) {
                    e.printStackTrace();
                }

                if (socket == null) {
                    Log.i("FirstFragment", "Couldn't access server " + SERVER + ":" + PORT);
                } else {
// read stream from server
                    byte[] buffer = new byte[256];
                    while (true) {
                        int bytes_read = 0;
                        try {
                            bytes_read = socket.getInputStream().read(buffer);


                        } catch (IOException e) {
                            e.printStackTrace();
                        }

                    }
                }
            }

        }

}