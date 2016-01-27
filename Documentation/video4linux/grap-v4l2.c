/*
 * grap-v4l2.c
 *
 *  Created on: 20.01.2010
 *      Author: euteneuer
 */

/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev2.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

#define PICWIDTH	1280 /*640*/
#define PICHEIGHT	1024 /*480*/
#define PICTOP		0
#define PICLEFT		0
#define PICBUFFERS	4


typedef enum {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
} io_method;

struct buffer {
        void *                  start;
        size_t                  length;
};

static char *           dev_name        = NULL;
static io_method        io              = IO_METHOD_MMAP;
static int              fd              = -1;
struct buffer *         buffers         = NULL;
static unsigned int     n_buffers       = 0;

static void
errno_exit                      (const char *           s)
{
        fprintf (stderr, "%s error %d, %s\n",
                 s, errno, strerror (errno));

        exit (EXIT_FAILURE);
}

static int
xioctl                          (int                    fd,
                                 int                    request,
                                 void *                 arg)
{
        int r;

        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);

        return r;
}

static void
process_image(const void * p,int length)
{
		unsigned short int *c = (unsigned short int *)p;
		char grey[]="@#8Oo;. ";
		//char grey[]=" .:oO8@";
		float g,gn = 8./1023; ///255;
		int i,s=0,mx,mn;
		int row,col;
		int width=PICWIDTH;
		int height=PICHEIGHT;
		int colstep=width/120;
		int rowstep=2*colstep;
		i=0;
		fprintf(stdout,"buffer @ 0x%08x\n",(int) p);
		fprintf(stdout,"width  %7d\n",width);
		fprintf(stdout,"height %7d\n",height);
		fprintf(stdout,"%c[H",27);
		mx=0;mn=65000;
		for (row=0;row<height;row+=rowstep) {
			i = row*width;
			fprintf(stdout,"%07d:",i);
			for (col=0; col<width;col+=colstep) {
				if (i>=length) break;
				if (c[i]>mx) mx=c[i];
				if (c[i]<mn) mn=c[i];
				g = gn*c[i];
				fputc(grey[(int) g],stdout);
				i+=colstep;
			}
			fputc('\n',stdout);
			fflush (stdout);
		}

		fprintf(stdout,"maximum %d        \n",mx);
		fprintf(stdout,"minimum %d        \n",mn);
}

static int
read_frame                      (void)
{
        struct v4l2_buffer buf;
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                if (-1 == read (fd, buffers[0].start, buffers[0].length)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */

                        default:
                                errno_exit ("read");
                        }
                }

                process_image (buffers[0].start,buffers[0].length);

                break;

        case IO_METHOD_MMAP:
                CLEAR (buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */

                        default:
                                errno_exit ("VIDIOC_DQBUF");
                        }
                }

                assert (buf.index < n_buffers);

                process_image (buffers[buf.index].start,buffers[0].length);
                //sleep(2);
                if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                        errno_exit ("VIDIOC_QBUF");

                break;

        case IO_METHOD_USERPTR:
                CLEAR (buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;

                if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */

                        default:
                                errno_exit ("VIDIOC_DQBUF");
                        }
                }

                for (i = 0; i < n_buffers; ++i)
                        if (buf.m.userptr == (unsigned long) buffers[i].start
                            && buf.length == buffers[i].length)
                                break;

                assert (i < n_buffers);

                process_image ((void *) buf.m.userptr,buf.length);

                if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                        errno_exit ("VIDIOC_QBUF");

                break;
        }

        return 1;
}

static void
mainloop                        (void)
{
        unsigned int count;

        count = 100;

        while (count-- > 0) {

                for (;;) {
                        fd_set fds;
                        struct timeval tv;
                        int r;

                        FD_ZERO (&fds);
                        FD_SET (fd, &fds);

                        /* Timeout. */
                        tv.tv_sec = 20;
                        tv.tv_usec = 0;

                        r = select (fd + 1, &fds, NULL, NULL, &tv);

                        if (-1 == r) {
                                if (EINTR == errno)
                                        continue;

                                errno_exit ("select");
                        }

                        if (0 == r) {
                                fprintf (stderr, "select timeout\n");
                                exit (EXIT_FAILURE);
                        }
                        fprintf (stderr, "reading frame ...\n");
                        //sleep(1);
                        if (read_frame ())
                                break;
                        fprintf (stderr, "...done ;)\n");


                        /* EAGAIN - continue select loop. */
                }
        }
}

static void
stop_capturing                  (void)
{
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
                        errno_exit ("VIDIOC_STREAMOFF");

                break;
        }
}

static void
start_capturing                 (void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
				fprintf(stderr,"attributing %d buffers ...\n",n_buffers);
                for (i = 0; i < n_buffers; i++) {
                        struct v4l2_buffer buf;
                        fprintf(stderr,"attributing buffer %d ",i);
                        CLEAR (buf);
                        fprintf(stderr,"..");
                        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory      = V4L2_MEMORY_MMAP;
                        buf.index       = i;
                        fprintf(stderr,"..");
                        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                                errno_exit ("VIDIOC_QBUF");
                        fprintf(stderr,"..");
                        fprintf(stderr,"done\n");
                }

                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                fprintf(stderr,"starting stream ...");

                if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
                        errno_exit ("VIDIOC_STREAMON");
                fprintf(stderr,"done\n");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR (buf);

                        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory      = V4L2_MEMORY_USERPTR;
                        buf.index       = i;
                        buf.m.userptr   = (unsigned long) buffers[i].start;
                        buf.length      = buffers[i].length;

                        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                                errno_exit ("VIDIOC_QBUF");
                }

                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
                        errno_exit ("VIDIOC_STREAMON");

                break;
        }
}

static void
uninit_device                   (void)
{
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                free (buffers[0].start);
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap (buffers[i].start, buffers[i].length))
                                errno_exit ("munmap");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i)
                        free (buffers[i].start);
                break;
        }

        free (buffers);
}

static void
init_read                       (unsigned int           buffer_size)
{
        buffers = calloc (1, sizeof (*buffers));

        if (!buffers) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc (buffer_size);

        if (!buffers[0].start) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }
}

static void
init_mmap                       (void)
{
        struct v4l2_requestbuffers req;

        CLEAR (req);

        req.count               = PICBUFFERS;
        req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory              = V4L2_MEMORY_MMAP;

        fprintf(stderr,"requesting buffer ...\n");
        if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf (stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit (EXIT_FAILURE);
                } else {
                        errno_exit ("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 1) {
                fprintf (stderr, "Insufficient buffer memory on %s\n",
                         dev_name);
                exit (EXIT_FAILURE);
        }
        fprintf(stderr,"calloc ...\n");
        buffers = calloc (req.count, sizeof (*buffers));

        if (!buffers) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }
        fprintf(stderr,"preparing buffers ...\n");
        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR (buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit ("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap (NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit ("mmap");
        }
}

static void
init_userp                      (unsigned int           buffer_size)
{
        struct v4l2_requestbuffers req;
        unsigned int page_size;

        page_size = getpagesize ();
        buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

        CLEAR (req);

        req.count               = 4;
        req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory              = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf (stderr, "%s does not support "
                                 "user pointer i/o\n", dev_name);
                        exit (EXIT_FAILURE);
                } else {
                        errno_exit ("VIDIOC_REQBUFS");
                }
        }

        buffers = calloc (4, sizeof (*buffers));

        if (!buffers) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = memalign (/* boundary */ page_size,
                                                     buffer_size);

                if (!buffers[n_buffers].start) {
                        fprintf (stderr, "Out of memory\n");
                        exit (EXIT_FAILURE);
                }
        }
}

static void
init_device                     (void)
{
        struct v4l2_capability cap;
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        struct v4l2_format fmt;
        unsigned int min;

        if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        fprintf (stderr, "%s is no V4L2 device\n",
                                 dev_name);
                        exit (EXIT_FAILURE);
                } else {
                        errno_exit ("VIDIOC_QUERYCAP");
                }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                fprintf (stderr, "%s is no video capture device\n",
                         dev_name);
                exit (EXIT_FAILURE);
        }

        switch (io) {
        case IO_METHOD_READ:
                if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
                        fprintf (stderr, "%s does not support read i/o\n",
                                 dev_name);
                        exit (EXIT_FAILURE);
                }

                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                        fprintf (stderr, "%s does not support streaming i/o\n",
                                 dev_name);
                        exit (EXIT_FAILURE);
                }

                break;
        }


        /* Select video input, video standard and tune here. */

        fprintf(stderr,"setting crop capabilities ...\n");
        CLEAR (cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect; /* reset to default */
                crop.c.width=PICWIDTH;
                crop.c.height=PICHEIGHT;
                crop.c.top=PICTOP;
                crop.c.left=PICLEFT;
                if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
                        switch (errno) {
                        case EINVAL:
								fprintf(stderr,"invalid cropping?\n");
                                /* Cropping not supported. */
                                break;
                        default:
								fprintf(stderr,"cropping error?\n");
                                /* Errors ignored. */
                                break;
                        }
                }
        } else {
        	fprintf(stderr,"no crop capabilities?\n");
                /* Errors ignored. */
        }

        fprintf(stderr,"setting format ...\n");
        CLEAR (fmt);

        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = PICWIDTH;
        fmt.fmt.pix.height      = PICHEIGHT;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE ;

        if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt))
                errno_exit ("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
        fprintf(stderr,"calling mode specific init %d x %d, %d Bpl, %d size ...\n",fmt.fmt.pix.width,fmt.fmt.pix.height,fmt.fmt.pix.bytesperline,fmt.fmt.pix.sizeimage);

        /* Buggy driver paranoia. */
        min = fmt.fmt.pix.width; // * 2;
        if (fmt.fmt.pix.bytesperline < min)
                fmt.fmt.pix.bytesperline = min;
        min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        if (fmt.fmt.pix.sizeimage < min)
                fmt.fmt.pix.sizeimage = min;

        fprintf(stderr,"calling mode specific init %d x %d, %d Bpl, %d size ...\n",fmt.fmt.pix.width,fmt.fmt.pix.height,fmt.fmt.pix.bytesperline,fmt.fmt.pix.sizeimage);
        switch (io) {
        case IO_METHOD_READ:
                init_read (fmt.fmt.pix.sizeimage);
                break;

        case IO_METHOD_MMAP:
                init_mmap ();
                break;

        case IO_METHOD_USERPTR:
                init_userp (fmt.fmt.pix.sizeimage);
                break;
        }
}

static void
close_device                    (void)
{
        if (-1 == close (fd))
                errno_exit ("close");

        fd = -1;
}

static void
open_device                     (void)
{
        struct stat st;

        if (-1 == stat (dev_name, &st)) {
                fprintf (stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror (errno));
                exit (EXIT_FAILURE);
        }

        if (!S_ISCHR (st.st_mode)) {
                fprintf (stderr, "%s is no device\n", dev_name);
                exit (EXIT_FAILURE);
        }

        fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf (stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror (errno));
                exit (EXIT_FAILURE);
        }
}

static void
usage                           (FILE *                 fp,
                                 int                    argc,
                                 char **                argv)
{
        fprintf (fp,
                 "Usage: %s [options]\n\n"
                 "Options:\n"
                 "-d | --device name   Video device name [/dev/video]\n"
                 "-h | --help          Print this message\n"
                 "-m | --mmap          Use memory mapped buffers\n"
                 "-r | --read          Use read() calls\n"
                 "-u | --userp         Use application allocated buffers\n"
                 "",
                 argv[0]);
}

static const char short_options [] = "d:hmru";

static const struct option
long_options [] = {
        { "device",     required_argument,      NULL,           'd' },
        { "help",       no_argument,            NULL,           'h' },
        { "mmap",       no_argument,            NULL,           'm' },
        { "read",       no_argument,            NULL,           'r' },
        { "userp",      no_argument,            NULL,           'u' },
        { 0, 0, 0, 0 }
};

int
main                            (int                    argc,
                                 char **                argv)
{
        dev_name = "/dev/video0";

        for (;;) {
                int index;
                int c;

                c = getopt_long (argc, argv,
                                 short_options, long_options,
                                 &index);

                if (-1 == c)
                        break;

                switch (c) {
                case 0: /* getopt_long() flag */
                        break;

                case 'd':
                        dev_name = optarg;
                        break;

                case 'h':
                        usage (stdout, argc, argv);
                        exit (EXIT_SUCCESS);

                case 'm':
                        io = IO_METHOD_MMAP;
                        break;

                case 'r':
                        io = IO_METHOD_READ;
                        break;

                case 'u':
                        io = IO_METHOD_USERPTR;
                        break;

                default:
                        usage (stderr, argc, argv);
                        exit (EXIT_FAILURE);
                }
        }
        fprintf(stderr,"opening device ...\n");
        open_device ();
        fprintf(stderr,"initializing device ...\n");
        init_device ();
        fprintf(stderr,"starting capture ...\n");
        start_capturing ();
        fprintf(stderr,"starting mainloop ...\n");
        mainloop ();
        fprintf(stderr,"stopping capture ...\n");
        stop_capturing ();
        fprintf(stderr,"uninitializin device ...\n");
        uninit_device ();
        fprintf(stderr,"closing device ...\n");
        close_device ();
        fprintf(stderr,"bye.\n");
        exit (EXIT_SUCCESS);

        return 0;
}
