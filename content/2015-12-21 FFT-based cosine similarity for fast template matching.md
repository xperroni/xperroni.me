Title: FFT-based cosine similarity for fast template matching
Date: 2015-12-21 22:51
Category: Code
Tags: Computer Vision
Authors: Helio Perroni Filho
Summary: How to perform fast image template matching using Finite Fourier Transforms

A common task in computer vision applications is [template matching](https://en.wikipedia.org/wiki/Template_matching), the search for a comparatively small image patch – or more likely, its closest approximation – in a larger image. In its simplest form, this implies solving the following optimization:

\begin{equation}
(i^*, j^*) = \arg \max_{i,j} s(T, P_{i,j})
\label{eq:tm}
\end{equation}

Where the similarity metric $s(T, P_{i,j})$ is defined between a template $T^{m_t \times n_t}$ and all patches $P^{m_t \times n_t}_{i,j}$ such that:

\begin{equation}
P^{m_t \times n_t}_{i,j} = I[i:i+m_t,j:j+n_t]
\label{eq:p}
\end{equation}

For an image $I^{m \times n}$ with $m_t \ll m$ and $n_t \ll n$ (i.e. the image is much larger than the template). A common choice of similarity metric is [cosine similarity](https://en.wikipedia.org/wiki/Cosine_similarity), which is defined as:

\begin{equation}
cos(A, B) = \frac{A \cdot B}{\|A\| \|B\|} = \frac{\displaystyle\sum^{m,n}_{i,j} A[i,j]B[i,j]}{\sqrt{\displaystyle\sum^{m,n}_{i,j} A[i,j]^2 \displaystyle\sum^{m,n}_{i,j} B[i,j]^2}}
\label{eq:cos}
\end{equation}

Cosine similarity is a reliable metric, but as defined above its evaluation is rather costly. Combining formulas \eqref{eq:tm}, \eqref{eq:p} and \eqref{eq:cos} gives:

\begin{equation}
(i^*, j^*) = \arg \max_{i,j} \frac{\displaystyle\sum^{m_t,n_t}_{i_t,j_t} T[i_t,j_t] I[i + i_t, j + j_t]}{\sqrt{\displaystyle\sum^{m_t,n_t}_{i_t,j_t} T[i_t,j_t]^2 \displaystyle\sum^{m_t,n_t}_{i_t,j_t} I[i + i_t, j + j_t]^2}}
\label{eq:tm_cos}
\end{equation}

Which is of time complexity $\mathrm{O}(m \, n \, m_tn_t)$ – i.e. the size of the template times the size of the image. Depending on image and template sizes, the intended use case (e.g. batch vs. real-time) or scale of the template matching task (e.g. a couple dozen vs. thousands of images), this may incur in prohibitive processing costs.

Cosine similarity template matching can be sped up with application of the [convolution theorem](https://en.wikipedia.org/wiki/Convolution_theorem). First, let's redefine formula \eqref{eq:tm_cos} as:

\begin{equation}
(i^*, j^*) = \arg \max_{i,j} \frac{D_{T, I}[i,j]}{\sqrt{m_T M_I[i,j]}}
\label{eq:tm_cos_2}
\end{equation}

Where:

\begin{equation}
D_{T, I}[i,j] = \displaystyle\sum^{m_t,n_t}_{i_t,j_t} T[i_t,j_t] I[i + i_t, j + j_t]
\label{eq:D_TI}
\end{equation}

\begin{equation}
m_T = \displaystyle\sum^{m_t,n_t}_{i_t,j_t} T[i_t,j_t]^2
\label{eq:m_T}
\end{equation}

\begin{equation}
M_I[i,j] = \displaystyle\sum^{m_t,n_t}_{i_t,j_t} I[i + i_t, j + j_t]^2
\label{eq:M_I}
\end{equation}

Looking into the three terms above, it's clear that \eqref{eq:m_T} is constant for any given $T$ and can therefore be left out of the computation. On the other hand, \eqref{eq:D_TI} is just the [cross-correlation](https://en.wikipedia.org/wiki/Cross-correlation) between $T$ and $I$, which by the convolution theorem can also be computed as:

\begin{equation}
D_{T, I} = \mathcal{F}^{-1}(\mathcal{F}(T)^{*} \circ \mathcal{F}(I))
\end{equation}

Where $\mathcal{F}$ is the [fourier transform](https://en.wikipedia.org/wiki/Fourier_transform) operator, $\mathcal{F}^{-1}$ the inverse transform, the asterisk denotes the [complex conjugate](https://en.wikipedia.org/wiki/Complex_conjugate), and $\circ$ is the [Hadamard product](https://en.wikipedia.org/wiki/Hadamard_product_%28matrices%29) (element-wise multiplication) of the two transforms. Likewise, \eqref{eq:M_I} can be computed as the cross-correlation between $I^2$ and a window filter $W^{m_t \times n_t} = [w_{ij} = 1]$:

\begin{equation}
M_I = \mathcal{F}^{-1}(\mathcal{F}(W)^{*} \circ \mathcal{F}(I^2))
\end{equation}

The advantage of this approach is that algorithms such as the [Fast Fourier Transform](https://en.wikipedia.org/wiki/Fast_Fourier_transform) (FFT) are of time complexity $\mathrm{O}(m \, n \, log \, m \, n)$, which depending on the relative sizes of $T$ and $I$ may be faster than the $\mathrm{O}(m \, n \, m_t n_t)$ of the direct computation. Also the Fourier transforms of $W$ and (if the same template will be applied to several images) $T$ can be cached, further saving up computation time.

<b>Implementation</b>


The C++ code below provides a basic implementation of the method outlined above. It uses the popular [OpenCV](http://opencv.org/) library, with some further optimizations particular to its implementation of the Fourier transform, explained in the comments.

    #include &lt;opencv2/opencv.hpp>

    static const cv::Scalar ONE(1);

    static const cv::Scalar ZERO(0);

    static const cv::Scalar WHITE(255, 255, 255);

    // Fourier transform performance is not a monotonic function of a vector
    // size - matrices whose dimensions are powers of two are the fastest to
    // process, and multiples of 2, 3 and 5 (for example, 300 = 5*5*3*2*2) are
    // also processed quite efficiently. Therefore it makes sense to pad input
    // data with zeros to get a bit larger matrix that can be transformed much
    // faster than the original one.
    cv::Size fit(const cv::Size &size)
    {
      return cv::Size(cv::getOptimalDFTSize(size.width),
                      cv::getOptimalDFTSize(size.height));
    }

    cv::Mat F_fwd(const cv::Mat &I, const cv::Size &size)
    {
      // Pad input matrix to given size.
      cv::Mat P;
      int m = size.height - I.rows;
      int n = size.width - I.cols;
      cv::copyMakeBorder(I, P, 0, m, 0, n, cv::BORDER_CONSTANT, ZERO);

      // Compute Fourier transform for input data. The last argument
      // informs the dft() function of how many non-zero rows are there,
      // so it can handle the rest of the rows more efficiently and save
      // some time.
      cv::Mat F;
      cv::dft(P, F, 0, I.rows);

      return F;
    }

    cv::Mat F_inv(const cv::Mat &F, const cv::Size &size)
    {
      // Compute inverse Fourier transform for input data. The last
      // argument informs the dft() function of how many non-zero
      // rows are expected in the output, so it can handle the rest
      // of the rows more efficiently and save some time.
      cv::Mat I;
      cv::dft(F, I, cv::DFT_INVERSE + cv::DFT_SCALE, size.height);

      return I(cv::Rect(0, 0, size.width, size.height));
    }

    cv::Mat C(const cv::Mat &T, const cv::Mat &I, const cv::Size &size)
    {
      // Compute the Fourier transforms of template and image.
      cv::Mat F_T = F_fwd(T, size);
      cv::Mat F_I = F_fwd(I, size);

      // Compute the cross correlation in the frequency domain.
      cv::Mat F_TI;
      cv::mulSpectrums(F_I, F_T, F_TI, 0, true);

      // Compute the inverse Fourier transform of the cross-correlation,
      // dismissing those rows and columns of the cross-correlation
      // matrix that would require the template to "roll over" the image.
      cv::Size clipped;
      clipped.width = I.cols - T.cols;
      clipped.height = I.rows - T.rows;
      return F_inv(F_TI, clipped);
    }

    cv::Mat W(const cv::Mat &T)
    {
      return cv::Mat(T.size(), CV_64F, ONE);
    }

    cv::Point3f matchTemplate(const cv::Mat &T, const cv::Mat &I)
    {
      // Compute the optimal size for DFT computing.
      cv::Size size = fit(I.size());

      //Compute the cross-correlation and normalizing matrix.
      cv::Mat C_TI = C(T, I, size);
      cv::Mat M_I = C(W(T), I.mul(I), size);

      int i_s, j_s;
      float r = 0;
      int rows = C_TI.rows;
      int cols = C_TI.cols;
      for (int i = 0; i < rows; i++)
      {
        for (int j = 0; j < cols; j++)
        {
          float v = C_TI.at<double>(i, j) / sqrt(M_I.at<double>(i, j));
          if (r < v)
          {
            r = v;
            i_s = i;
            j_s = j;
          }
        }
      }

      return cv::Point3f(j_s, i_s, r);
    }

    cv::Mat L(const cv::Mat &I)
    {
      cv::Mat L_I, L_F;
      cv::cvtColor(I, L_I, CV_BGR2GRAY);
      L_I.convertTo(L_F, CV_64F);
      return L_F;
    }

    int main(int argc, char *argv[])
    {
      cv::Mat T = cv::imread(argv[1]);
      cv::Mat I = cv::imread(argv[2]);

      cv::Point3f match = matchTemplate(L(T), L(I));
      cv::rectangle(I, cv::Rect(match.x, match.y, T.cols, T.rows), WHITE);

      cv::imshow("Template", T);
      cv::imshow("Image", I);
      cv::waitKey();

      return 0;
    }
