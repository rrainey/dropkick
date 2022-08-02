This is a test of using LaTex embedded in GitHub markdown.



$$
\begin{bmatrix}
\dot{q_0} \\
\dot{q_1} \\
\dot{q_2} \\
\dot{q_3}
\end{bmatrix} = - \{ 1 \over 2 \}  \begin{bmatrix*}[r] 
 0  &  P  &  Q  &  R \\
-P  &  0  & -R  & Q \\
-Q  &  R  & 0   & -P \\
-R  & -Q  &  P  &  0 \\
\end{bmatrix*}
\begin{bmatrix}
q_0 \\
q_1 \\
q_2 \\
q_3
\end{bmatrix}
$$

From ANSI/AIAA R-004-1992:

$$
\begin{matrix}
\\
\theta = \arcsin(-2 ( q_1 q_3 - q_0 q_2 )) \\
\\
\psi = \{ \arctan( \{ 2( q_1 q_2 + q_0 q_3 ) \over ( q_0^2 + q_1^2 - q_2^2 - q_3^2 ) \} ) \} \\
\\
\phi = \{ \arctan( \{ 2( q_2 q_3 + q_0 q_1 ) \over ( q_0^2 + q_3^2 - q_1^2 - q_2^2 ) \} ) \} \\
\end{matrix}
$$
