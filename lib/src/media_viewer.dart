import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/physics.dart';
import 'package:flutter/widgets.dart';
import 'package:vector_math/vector_math_64.dart';

class MediaViewer extends StatefulWidget {
  /// Create a MediaViewer.
  MediaViewer({
    super.key,
    this.clipBehavior = Clip.hardEdge,
    this.panAxis = PanAxis.free,
    // These default scale values were eyeballed as reasonable limits for common
    // use cases.
    this.maxScale = 2.5,
    this.minScale = 1,
    this.interactionEndFrictionCoefficient = _kDrag,
    this.onInteractionEnd,
    this.onInteractionStart,
    this.onInteractionUpdate,
    this.panEnabled = true,
    this.scaleEnabled = true,
    this.scaleFactor = kDefaultMouseScrollToScaleFactor,
    this.transformationController,
    this.trackpadScrollCausesScale = false,
    required this.child,
  })  : assert(minScale > 0),
        assert(interactionEndFrictionCoefficient > 0),
        assert(minScale.isFinite),
        assert(maxScale > 0),
        assert(!maxScale.isNaN),
        assert(maxScale >= minScale);

  /// If set to [Clip.none], the child may extend beyond the size of the MediaViewer,
  /// but it will not receive gestures in these areas.
  /// Be sure that the MediaViewer is the desired size when using [Clip.none].
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// When set to [PanAxis.aligned], panning is only allowed in the horizontal
  /// axis or the vertical axis, diagonal panning is not allowed.
  ///
  /// When set to [PanAxis.vertical] or [PanAxis.horizontal] panning is only
  /// allowed in the specified axis. For example, if set to [PanAxis.vertical],
  /// panning will only be allowed in the vertical axis. And if set to [PanAxis.horizontal],
  /// panning will only be allowed in the horizontal axis.
  ///
  /// When set to [PanAxis.free] panning is allowed in all directions.
  ///
  /// Defaults to [PanAxis.free].
  final PanAxis panAxis;

  /// The child [Widget] that is transformed by MediaViewer.
  final Widget child;

  /// If false, the user will be prevented from panning.
  ///
  /// Defaults to true.
  ///
  /// See also:
  ///
  ///   * [scaleEnabled], which is similar but for scale.
  final bool panEnabled;

  /// If false, the user will be prevented from scaling.
  ///
  /// Defaults to true.
  ///
  /// See also:
  ///
  ///   * [panEnabled], which is similar but for panning.
  final bool scaleEnabled;

  /// {@macro flutter.gestures.scale.trackpadScrollCausesScale}
  final bool trackpadScrollCausesScale;

  /// Determines the amount of scale to be performed per pointer scroll.
  ///
  /// Defaults to [kDefaultMouseScrollToScaleFactor].
  ///
  /// Increasing this value above the default causes scaling to feel slower,
  /// while decreasing it causes scaling to feel faster.
  ///
  /// The amount of scale is calculated as the exponential function of the
  /// [PointerScrollEvent.scrollDelta] to [scaleFactor] ratio. In the Flutter
  /// engine, the mousewheel [PointerScrollEvent.scrollDelta] is hardcoded to 20
  /// per scroll, while a trackpad scroll can be any amount.
  ///
  /// Affects only pointer device scrolling, not pinch to zoom.
  final double scaleFactor;

  /// The maximum allowed scale.
  ///
  /// The scale will be clamped between this and [minScale] inclusively.
  ///
  /// Defaults to 2.5.
  ///
  /// Must be greater than zero and greater than [minScale].
  final double maxScale;

  /// The minimum allowed scale.
  ///
  /// The scale will be clamped between this and [maxScale] inclusively.
  ///
  /// Scale is also affected by [boundaryMargin]. If the scale would result in
  /// viewing beyond the boundary, then it will not be allowed. By default,
  /// boundaryMargin is EdgeInsets.zero, so scaling below 1.0 will not be
  /// allowed in most cases without first increasing the boundaryMargin.
  ///
  /// Defaults to 0.8.
  ///
  /// Must be a finite number greater than zero and less than [maxScale].
  final double minScale;

  /// Changes the deceleration behavior after a gesture.
  ///
  /// Defaults to 0.0000135.
  ///
  /// Must be a finite number greater than zero.
  final double interactionEndFrictionCoefficient;

  /// Called when the user ends a pan or scale gesture on the widget.
  ///
  /// At the time this is called, the [TransformationController] will have
  /// already been updated to reflect the change caused by the interaction,
  /// though a pan may cause an inertia animation after this is called as well.
  ///
  /// {@template flutter.widgets.InteractiveViewer.onInteractionEnd}
  /// Will be called even if the interaction is disabled with [panEnabled] or
  /// [scaleEnabled] for both touch gestures and mouse interactions.
  ///
  /// A [GestureDetector] wrapping the MediaViewer will not respond to
  /// [GestureDetector.onScaleStart], [GestureDetector.onScaleUpdate], and
  /// [GestureDetector.onScaleEnd]. Use [onInteractionStart],
  /// [onInteractionUpdate], and [onInteractionEnd] to respond to those
  /// gestures.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [onInteractionStart], which handles the start of the same interaction.
  ///  * [onInteractionUpdate], which handles an update to the same interaction.
  final GestureScaleEndCallback? onInteractionEnd;

  /// Called when the user begins a pan or scale gesture on the widget.
  ///
  /// At the time this is called, the [TransformationController] will not have
  /// changed due to this interaction.
  ///
  /// {@macro flutter.widgets.InteractiveViewer.onInteractionEnd}
  ///
  /// The coordinates provided in the details' `focalPoint` and
  /// `localFocalPoint` are normal Flutter event coordinates, not
  /// MediaViewer scene coordinates. See
  /// [TransformationController.toScene] for how to convert these coordinates to
  /// scene coordinates relative to the child.
  ///
  /// See also:
  ///
  ///  * [onInteractionUpdate], which handles an update to the same interaction.
  ///  * [onInteractionEnd], which handles the end of the same interaction.
  final GestureScaleStartCallback? onInteractionStart;

  /// Called when the user updates a pan or scale gesture on the widget.
  ///
  /// At the time this is called, the [TransformationController] will have
  /// already been updated to reflect the change caused by the interaction, if
  /// the interaction caused the matrix to change.
  ///
  /// {@macro flutter.widgets.InteractiveViewer.onInteractionEnd}
  ///
  /// The coordinates provided in the details' `focalPoint` and
  /// `localFocalPoint` are normal Flutter event coordinates, not
  /// MediaViewer scene coordinates. See
  /// [TransformationController.toScene] for how to convert these coordinates to
  /// scene coordinates relative to the child.
  ///
  /// See also:
  ///
  ///  * [onInteractionStart], which handles the start of the same interaction.
  ///  * [onInteractionEnd], which handles the end of the same interaction.
  final GestureScaleUpdateCallback? onInteractionUpdate;

  /// A [TransformationController] for the transformation performed on the
  /// child.
  ///
  /// Whenever the child is transformed, the [Matrix4] value is updated and all
  /// listeners are notified. If the value is set, MediaViewer will update
  /// to respect the new value.
  ///
  /// {@tool dartpad}
  /// This example shows how transformationController can be used to animate the
  /// transformation back to its starting position.
  ///
  /// ** See code in examples/api/lib/widgets/interactive_viewer/interactive_viewer.transformation_controller.0.dart **
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [ValueNotifier], the parent class of TransformationController.
  ///  * [TextEditingController] for an example of another similar pattern.
  final TransformationController? transformationController;

  // Used as the coefficient of friction in the inertial translation animation.
  // This value was eyeballed to give a feel similar to Google Photos.
  static const double _kDrag = 0.0000135;

  /// Returns the closest point to the given point on the given line segment.
  @visibleForTesting
  static Vector3 getNearestPointOnLine(Vector3 point, Vector3 l1, Vector3 l2) {
    final double lengthSquared = math.pow(l2.x - l1.x, 2.0).toDouble() +
        math.pow(l2.y - l1.y, 2.0).toDouble();

    // In this case, l1 == l2.
    if (lengthSquared == 0) {
      return l1;
    }

    // Calculate how far down the line segment the closest point is and return
    // the point.
    final Vector3 l1P = point - l1;
    final Vector3 l1L2 = l2 - l1;
    final double fraction =
        clampDouble(l1P.dot(l1L2) / lengthSquared, 0.0, 1.0);
    return l1 + l1L2 * fraction;
  }

  /// Given a quad, return its axis aligned bounding box.
  @visibleForTesting
  static Quad getAxisAlignedBoundingBox(Quad quad) {
    final double minX = math.min(
      quad.point0.x,
      math.min(
        quad.point1.x,
        math.min(
          quad.point2.x,
          quad.point3.x,
        ),
      ),
    );
    final double minY = math.min(
      quad.point0.y,
      math.min(
        quad.point1.y,
        math.min(
          quad.point2.y,
          quad.point3.y,
        ),
      ),
    );
    final double maxX = math.max(
      quad.point0.x,
      math.max(
        quad.point1.x,
        math.max(
          quad.point2.x,
          quad.point3.x,
        ),
      ),
    );
    final double maxY = math.max(
      quad.point0.y,
      math.max(
        quad.point1.y,
        math.max(
          quad.point2.y,
          quad.point3.y,
        ),
      ),
    );
    return Quad.points(
      Vector3(minX, minY, 0),
      Vector3(maxX, minY, 0),
      Vector3(maxX, maxY, 0),
      Vector3(minX, maxY, 0),
    );
  }

  /// Returns true iff the point is inside the rectangle given by the Quad,
  /// inclusively.
  /// Algorithm from https://math.stackexchange.com/a/190373.
  @visibleForTesting
  static bool pointIsInside(Vector3 point, Quad quad) {
    final Vector3 aM = point - quad.point0;
    final Vector3 aB = quad.point1 - quad.point0;
    final Vector3 aD = quad.point3 - quad.point0;

    final double aMAB = aM.dot(aB);
    final double aBAB = aB.dot(aB);
    final double aMAD = aM.dot(aD);
    final double aDAD = aD.dot(aD);

    return 0 <= aMAB && aMAB <= aBAB && 0 <= aMAD && aMAD <= aDAD;
  }

  /// Get the point inside (inclusively) the given Quad that is nearest to the
  /// given Vector3.
  @visibleForTesting
  static Vector3 getNearestPointInside(Vector3 point, Quad quad) {
    // If the point is inside the axis aligned bounding box, then it's ok where
    // it is.
    if (pointIsInside(point, quad)) {
      return point;
    }

    // Otherwise, return the nearest point on the quad.
    final List<Vector3> closestPoints = <Vector3>[
      MediaViewer.getNearestPointOnLine(point, quad.point0, quad.point1),
      MediaViewer.getNearestPointOnLine(point, quad.point1, quad.point2),
      MediaViewer.getNearestPointOnLine(point, quad.point2, quad.point3),
      MediaViewer.getNearestPointOnLine(point, quad.point3, quad.point0),
    ];
    double minDistance = double.infinity;
    late Vector3 closestOverall;
    for (final Vector3 closePoint in closestPoints) {
      final double distance = math.sqrt(
        math.pow(point.x - closePoint.x, 2) +
            math.pow(point.y - closePoint.y, 2),
      );
      if (distance < minDistance) {
        minDistance = distance;
        closestOverall = closePoint;
      }
    }
    return closestOverall;
  }

  @override
  State<MediaViewer> createState() => _MediaViewerState();
}

class _MediaViewerState extends State<MediaViewer>
    with TickerProviderStateMixin {
  TransformationController? _transformationController;

  GlobalKey _childKey = GlobalKey();
  final GlobalKey _parentKey = GlobalKey();
  Animation<Offset>? _animation;
  Animation<double>? _scaleAnimation;
  late Offset _scaleAnimationFocalPoint;
  late AnimationController _controller;
  late AnimationController _scaleController;
  Axis? _currentAxis; // Used with panAxis.
  Offset? _referenceFocalPoint; // Point where the current gesture began.
  double? _scaleStart; // Scale value at start of scaling gesture.
  _GestureType? _gestureType;

  bool get _boundarySizeIsValid {
    if (_childKey.currentContext?.findRenderObject()
        case RenderBox(size: final Size size)
        when !size.isEmpty && size.isFinite) {
      return true;
    }
    return false;
  }

  // The _boundaryRect is calculated by adding the boundaryMargin to the size of
  // the child.
  Rect get _boundaryRect {
    assert(_childKey.currentContext != null);

    final RenderBox childRenderBox =
        _childKey.currentContext!.findRenderObject()! as RenderBox;
    final Size childSize = childRenderBox.size;
    final Rect boundaryRect = Offset.zero & childSize;
    assert(
      !boundaryRect.isEmpty,
      "MediaViewer's child must have nonzero dimensions.",
    );
    // Boundaries that are partially infinite are not allowed because Matrix4's
    // rotation and translation methods don't handle infinites well.
    assert(
      boundaryRect.isFinite ||
          (boundaryRect.left.isInfinite &&
              boundaryRect.top.isInfinite &&
              boundaryRect.right.isInfinite &&
              boundaryRect.bottom.isInfinite),
      'boundaryRect must either be infinite in all directions or finite in all directions.',
    );
    return boundaryRect;
  }

  // The Rect representing the child's parent.
  Rect get _viewport {
    assert(_parentKey.currentContext != null);
    final RenderBox parentRenderBox =
        _parentKey.currentContext!.findRenderObject()! as RenderBox;
    return Offset.zero & parentRenderBox.size;
  }

  // Return a new matrix representing the given matrix after applying the given
  // translation.
  Matrix4 _matrixTranslate(Matrix4 matrix, Offset translation) {
    if (translation == Offset.zero) {
      return matrix.clone();
    }

    late final Offset alignedTranslation;

    if (_currentAxis != null) {
      switch (widget.panAxis) {
        case PanAxis.horizontal:
          alignedTranslation = _alignAxis(translation, Axis.horizontal);
        case PanAxis.vertical:
          alignedTranslation = _alignAxis(translation, Axis.vertical);
        case PanAxis.aligned:
          alignedTranslation = _alignAxis(translation, _currentAxis!);
        case PanAxis.free:
          alignedTranslation = translation;
      }
    } else {
      alignedTranslation = translation;
    }

    final Matrix4 nextMatrix = matrix.clone()
      ..translate(
        alignedTranslation.dx,
        alignedTranslation.dy,
      );

    // Transform the viewport to determine where its four corners will be after
    // the child has been transformed.
    final Quad nextViewport = _transformViewport(nextMatrix, _viewport);

    // If the boundaries are infinite, then no need to check if the translation
    // fits within them.
    if (_boundaryRect.isInfinite) {
      return nextMatrix;
    }

    // Get axis aligned boundary
    final Quad boundariesAabbQuad = _getAxisAlignedBoundingBox(_boundaryRect);

    // If the given translation fits completely within the boundaries, allow it.
    final Offset offendingDistance =
        _exceedsBy(boundariesAabbQuad, nextViewport);
    if (offendingDistance == Offset.zero) {
      return nextMatrix;
    }

    // Desired translation goes out of bounds, so translate to the nearest
    // in-bounds point instead.
    final Offset nextTotalTranslation = _getMatrixTranslation(nextMatrix);
    final double currentScale = matrix.getMaxScaleOnAxis();
    final Offset correctedTotalTranslation = Offset(
      nextTotalTranslation.dx - offendingDistance.dx * currentScale,
      nextTotalTranslation.dy - offendingDistance.dy * currentScale,
    );

    final Matrix4 correctedMatrix = matrix.clone()
      ..setTranslation(Vector3(
        correctedTotalTranslation.dx,
        correctedTotalTranslation.dy,
        0.0,
      ));

    // Double check that the corrected translation fits.
    final Quad correctedViewport =
        _transformViewport(correctedMatrix, _viewport);
    final Offset offendingCorrectedDistance =
        _exceedsBy(boundariesAabbQuad, correctedViewport);
    if (offendingCorrectedDistance == Offset.zero) {
      return correctedMatrix;
    }

    // If the corrected translation doesn't fit in either direction, don't allow
    // any translation at all. This happens when the viewport is larger than the
    // entire boundary.
    if (offendingCorrectedDistance.dx != 0.0 &&
        offendingCorrectedDistance.dy != 0.0) {
      return matrix.clone();
    }

    // Otherwise, allow translation in only the direction that fits. This
    // happens when the viewport is larger than the boundary in one direction.
    final Offset unidirectionalCorrectedTotalTranslation = Offset(
      offendingCorrectedDistance.dx == 0.0 ? correctedTotalTranslation.dx : 0.0,
      offendingCorrectedDistance.dy == 0.0 ? correctedTotalTranslation.dy : 0.0,
    );
    return matrix.clone()
      ..setTranslation(Vector3(
        unidirectionalCorrectedTotalTranslation.dx,
        unidirectionalCorrectedTotalTranslation.dy,
        0.0,
      ));
  }

  // Return a new matrix representing the given matrix after applying the given
  // scale.
  Matrix4 _matrixScale(Matrix4 matrix, double scale) {
    if (scale == 1.0) {
      return matrix.clone();
    }
    assert(scale != 0.0);

    // Don't allow a scale that results in an overall scale beyond min/max
    // scale.
    final double currentScale =
        _transformationController!.value.getMaxScaleOnAxis();
    final double totalScale = currentScale * scale;
    final double clampedTotalScale = clampDouble(
      totalScale,
      widget.minScale,
      widget.maxScale,
    );
    final double clampedScale = clampedTotalScale / currentScale;
    return matrix.clone()..scale(clampedScale);
  }

  // Returns true if the given _GestureType is enabled.
  bool _gestureIsSupported(_GestureType? gestureType) {
    switch (gestureType) {
      case _GestureType.scale:
        return widget.scaleEnabled;
      case _GestureType.pan:
      case null:
        return widget.panEnabled;
    }
  }

  // Decide which type of gesture this is by comparing the amount of scale
  // and rotation in the gesture, if any. Scale starts at 1 and rotation
  // starts at 0. Pan will have no scale and no rotation because it uses only one
  // finger.
  _GestureType _getGestureType(ScaleUpdateDetails details) {
    final double scale = !widget.scaleEnabled ? 1.0 : details.scale;
    if (scale != 1.0) {
      return _GestureType.scale;
    } else {
      return _GestureType.pan;
    }
  }

  // Handle the start of a gesture. All of pan, scale, and rotate are handled
  // with GestureDetector's scale gesture.
  void _onScaleStart(ScaleStartDetails details) {
    widget.onInteractionStart?.call(details);

    if (_controller.isAnimating) {
      _controller.stop();
      _controller.reset();
      _animation?.removeListener(_onAnimate);
      _animation = null;
    }
    if (_scaleController.isAnimating) {
      _scaleController.stop();
      _scaleController.reset();
      _scaleAnimation?.removeListener(_onScaleAnimate);
      _scaleAnimation = null;
    }

    _gestureType = null;
    _currentAxis = null;
    _scaleStart = _transformationController!.value.getMaxScaleOnAxis();
    _referenceFocalPoint =
        _transformationController!.toScene(details.localFocalPoint);
  }

  // Handle an update to an ongoing gesture. All of pan and scale are
  // handled with GestureDetector's scale gesture.
  void _onScaleUpdate(ScaleUpdateDetails details) {
    final double scale = _transformationController!.value.getMaxScaleOnAxis();
    _scaleAnimationFocalPoint = details.localFocalPoint;
    final Offset focalPointScene =
        _transformationController!.toScene(details.localFocalPoint);

    if (_gestureType == _GestureType.pan) {
      // When a gesture first starts, it sometimes has no change in scale and
      // rotation despite being a two-finger gesture. Here the gesture is
      // allowed to be reinterpreted as its correct type after originally
      // being marked as a pan.
      _gestureType = _getGestureType(details);
    } else {
      _gestureType ??= _getGestureType(details);
    }
    if (!_gestureIsSupported(_gestureType)) {
      widget.onInteractionUpdate?.call(details);
      return;
    }

    switch (_gestureType!) {
      case _GestureType.scale:
        assert(_scaleStart != null);
        // details.scale gives us the amount to change the scale as of the
        // start of this gesture, so calculate the amount to scale as of the
        // previous call to _onScaleUpdate.
        final double desiredScale = _scaleStart! * details.scale;
        final double scaleChange = desiredScale / scale;
        _transformationController!.value = _matrixScale(
          _transformationController!.value,
          scaleChange,
        );

        // While scaling, translate such that the user's two fingers stay on
        // the same places in the scene. That means that the focal point of
        // the scale should be on the same place in the scene before and after
        // the scale.
        final Offset focalPointSceneScaled =
            _transformationController!.toScene(details.localFocalPoint);

        _transformationController!.value = _matrixTranslate(
          _transformationController!.value,
          focalPointSceneScaled - _referenceFocalPoint!,
        );

        // details.localFocalPoint should now be at the same location as the
        // original _referenceFocalPoint point. If it's not, that's because
        // the translate came in contact with a boundary. In that case, update
        // _referenceFocalPoint so subsequent updates happen in relation to
        // the new effective focal point.
        final Offset focalPointSceneCheck =
            _transformationController!.toScene(details.localFocalPoint);
        if (_round(_referenceFocalPoint!) != _round(focalPointSceneCheck)) {
          _referenceFocalPoint = focalPointSceneCheck;
        }

      case _GestureType.pan:
        assert(_referenceFocalPoint != null);
        // details may have a change in scale here when scaleEnabled is false.
        // In an effort to keep the behavior similar whether or not scaleEnabled
        // is true, these gestures are thrown away.
        if (details.scale != 1.0) {
          widget.onInteractionUpdate?.call(details);
          return;
        }
        _currentAxis ??= _getPanAxis(_referenceFocalPoint!, focalPointScene);
        // Translate so that the same point in the scene is underneath the
        // focal point before and after the movement.
        final Offset translationChange =
            focalPointScene - _referenceFocalPoint!;
        _transformationController!.value = _matrixTranslate(
          _transformationController!.value,
          translationChange,
        );
        _referenceFocalPoint =
            _transformationController!.toScene(details.localFocalPoint);
    }
    widget.onInteractionUpdate?.call(details);
  }

  // Handle the end of a gesture of _GestureType. All of pan, scale, and rotate
  // are handled with GestureDetector's scale gesture.
  void _onScaleEnd(ScaleEndDetails details) {
    widget.onInteractionEnd?.call(details);
    _scaleStart = null;
    _referenceFocalPoint = null;

    _animation?.removeListener(_onAnimate);
    _scaleAnimation?.removeListener(_onScaleAnimate);
    _controller.reset();
    _scaleController.reset();

    if (!_gestureIsSupported(_gestureType)) {
      _currentAxis = null;
      return;
    }

    if (_gestureType == _GestureType.pan) {
      if (details.velocity.pixelsPerSecond.distance < kMinFlingVelocity) {
        _currentAxis = null;
        return;
      }
      final Vector3 translationVector =
          _transformationController!.value.getTranslation();
      final Offset translation =
          Offset(translationVector.x, translationVector.y);
      final FrictionSimulation frictionSimulationX = FrictionSimulation(
        widget.interactionEndFrictionCoefficient,
        translation.dx,
        details.velocity.pixelsPerSecond.dx,
      );
      final FrictionSimulation frictionSimulationY = FrictionSimulation(
        widget.interactionEndFrictionCoefficient,
        translation.dy,
        details.velocity.pixelsPerSecond.dy,
      );
      final double tFinal = _getFinalTime(
        details.velocity.pixelsPerSecond.distance,
        widget.interactionEndFrictionCoefficient,
      );
      _animation = Tween<Offset>(
        begin: translation,
        end: Offset(frictionSimulationX.finalX, frictionSimulationY.finalX),
      ).animate(CurvedAnimation(
        parent: _controller,
        curve: Curves.decelerate,
      ));
      _controller.duration = Duration(milliseconds: (tFinal * 1000).round());
      _animation!.addListener(_onAnimate);
      _controller.forward();
    } else if (_gestureType == _GestureType.scale) {
      if (details.scaleVelocity.abs() < 0.1) {
        _currentAxis = null;
        return;
      }
      final double scale = _transformationController!.value.getMaxScaleOnAxis();
      final FrictionSimulation frictionSimulation = FrictionSimulation(
          widget.interactionEndFrictionCoefficient * widget.scaleFactor,
          scale,
          details.scaleVelocity / 10);
      final double tFinal = _getFinalTime(
          details.scaleVelocity.abs(), widget.interactionEndFrictionCoefficient,
          effectivelyMotionless: 0.1);
      _scaleAnimation =
          Tween<double>(begin: scale, end: frictionSimulation.x(tFinal))
              .animate(CurvedAnimation(
                  parent: _scaleController, curve: Curves.decelerate));
      _scaleController.duration =
          Duration(milliseconds: (tFinal * 1000).round());
      _scaleAnimation!.addListener(_onScaleAnimate);
      _scaleController.forward();
    }
  }

  // Handle mousewheel and web trackpad scroll events.
  void _receivedPointerSignal(PointerSignalEvent event) {
    final double scaleChange;
    if (event is PointerScrollEvent) {
      if (event.kind == PointerDeviceKind.trackpad &&
          !widget.trackpadScrollCausesScale) {
        // Trackpad scroll, so treat it as a pan.
        widget.onInteractionStart?.call(
          ScaleStartDetails(
            focalPoint: event.position,
            localFocalPoint: event.localPosition,
          ),
        );

        final Offset localDelta = PointerEvent.transformDeltaViaPositions(
          untransformedEndPosition: event.position + event.scrollDelta,
          untransformedDelta: event.scrollDelta,
          transform: event.transform,
        );

        if (!_gestureIsSupported(_GestureType.pan)) {
          widget.onInteractionUpdate?.call(ScaleUpdateDetails(
            focalPoint: event.position - event.scrollDelta,
            localFocalPoint: event.localPosition - event.scrollDelta,
            focalPointDelta: -localDelta,
          ));
          widget.onInteractionEnd?.call(ScaleEndDetails());
          return;
        }

        final Offset focalPointScene =
            _transformationController!.toScene(event.localPosition);

        final Offset newFocalPointScene = _transformationController!
            .toScene(event.localPosition - localDelta);

        _transformationController!.value = _matrixTranslate(
            _transformationController!.value,
            newFocalPointScene - focalPointScene);

        widget.onInteractionUpdate?.call(ScaleUpdateDetails(
            focalPoint: event.position - event.scrollDelta,
            localFocalPoint: event.localPosition - localDelta,
            focalPointDelta: -localDelta));
        widget.onInteractionEnd?.call(ScaleEndDetails());
        return;
      }
      // Ignore left and right mouse wheel scroll.
      if (event.scrollDelta.dy == 0.0) {
        return;
      }
      scaleChange = math.exp(-event.scrollDelta.dy / widget.scaleFactor);
    } else if (event is PointerScaleEvent) {
      scaleChange = event.scale;
    } else {
      return;
    }
    widget.onInteractionStart?.call(
      ScaleStartDetails(
        focalPoint: event.position,
        localFocalPoint: event.localPosition,
      ),
    );

    if (!_gestureIsSupported(_GestureType.scale)) {
      widget.onInteractionUpdate?.call(ScaleUpdateDetails(
        focalPoint: event.position,
        localFocalPoint: event.localPosition,
        scale: scaleChange,
      ));
      widget.onInteractionEnd?.call(ScaleEndDetails());
      return;
    }

    final Offset focalPointScene =
        _transformationController!.toScene(event.localPosition);

    _transformationController!.value = _matrixScale(
      _transformationController!.value,
      scaleChange,
    );

    // After scaling, translate such that the event's position is at the
    // same scene point before and after the scale.
    final Offset focalPointSceneScaled =
        _transformationController!.toScene(event.localPosition);
    _transformationController!.value = _matrixTranslate(
      _transformationController!.value,
      focalPointSceneScaled - focalPointScene,
    );

    widget.onInteractionUpdate?.call(ScaleUpdateDetails(
      focalPoint: event.position,
      localFocalPoint: event.localPosition,
      scale: scaleChange,
    ));
    widget.onInteractionEnd?.call(ScaleEndDetails());
  }

  // Handle inertia drag animation.
  void _onAnimate() {
    if (!_controller.isAnimating) {
      _currentAxis = null;
      _animation?.removeListener(_onAnimate);
      _animation = null;
      _controller.reset();
      return;
    }
    // Translate such that the resulting translation is _animation.value.
    final Vector3 translationVector =
        _transformationController!.value.getTranslation();
    final Offset translation = Offset(translationVector.x, translationVector.y);
    final Offset translationScene =
        _transformationController!.toScene(translation);
    final Offset animationScene =
        _transformationController!.toScene(_animation!.value);
    final Offset translationChangeScene = animationScene - translationScene;
    _transformationController!.value = _matrixTranslate(
      _transformationController!.value,
      translationChangeScene,
    );
  }

  // Handle inertia scale animation.
  void _onScaleAnimate() {
    if (!_scaleController.isAnimating) {
      _currentAxis = null;
      _scaleAnimation?.removeListener(_onScaleAnimate);
      _scaleAnimation = null;
      _scaleController.reset();
      return;
    }
    final double desiredScale = _scaleAnimation!.value;
    final double scaleChange =
        desiredScale / _transformationController!.value.getMaxScaleOnAxis();
    final Offset referenceFocalPoint =
        _transformationController!.toScene(_scaleAnimationFocalPoint);
    _transformationController!.value = _matrixScale(
      _transformationController!.value,
      scaleChange,
    );

    // While scaling, translate such that the user's two fingers stay on
    // the same places in the scene. That means that the focal point of
    // the scale should be on the same place in the scene before and after
    // the scale.
    final Offset focalPointSceneScaled =
        _transformationController!.toScene(_scaleAnimationFocalPoint);
    _transformationController!.value = _matrixTranslate(
      _transformationController!.value,
      focalPointSceneScaled - referenceFocalPoint,
    );
  }

  void _onDoubleTapDown(TapDownDetails details) {
    _scaleAnimationFocalPoint =
        _transformationController!.toScene(details.localPosition);
  }

  void _onDoubleTap() {
    _scaleController.reset();
    final double scale = _transformationController!.value.getMaxScaleOnAxis();
    final double tFinal =
        scale - widget.minScale > 0.1 ? widget.minScale : widget.maxScale;
    _scaleAnimation = Tween<double>(begin: scale, end: tFinal).animate(
        CurvedAnimation(parent: _scaleController, curve: Curves.decelerate));
    _scaleController.duration = const Duration(milliseconds: 250);
    _scaleAnimation!.addListener(_onScaleAnimate);
    _scaleController.forward();
  }

  void _onTransformationControllerChange() {
    // A change to the TransformationController's value is a change to the
    // state.
    setState(() {});
  }

  @override
  void initState() {
    super.initState();

    _transformationController =
        widget.transformationController ?? TransformationController();
    _transformationController!.addListener(_onTransformationControllerChange);
    _controller = AnimationController(
      vsync: this,
    );
    _scaleController = AnimationController(vsync: this);
  }

  @override
  void didUpdateWidget(MediaViewer oldWidget) {
    super.didUpdateWidget(oldWidget);
    // Handle all cases of needing to dispose and initialize
    // transformationControllers.
    if (oldWidget.transformationController == null) {
      if (widget.transformationController != null) {
        _transformationController!
            .removeListener(_onTransformationControllerChange);
        _transformationController!.dispose();
        _transformationController = widget.transformationController;
        _transformationController!
            .addListener(_onTransformationControllerChange);
      }
    } else {
      if (widget.transformationController == null) {
        _transformationController!
            .removeListener(_onTransformationControllerChange);
        _transformationController = TransformationController();
        _transformationController!
            .addListener(_onTransformationControllerChange);
      } else if (widget.transformationController !=
          oldWidget.transformationController) {
        _transformationController!
            .removeListener(_onTransformationControllerChange);
        _transformationController = widget.transformationController;
        _transformationController!
            .addListener(_onTransformationControllerChange);
      }
    }
    if (oldWidget.child != widget.child) {
      // Ensure old child size is not used
      _childKey = GlobalKey();
    }
  }

  @override
  void dispose() {
    _controller.dispose();
    _scaleController.dispose();
    _transformationController!
        .removeListener(_onTransformationControllerChange);
    if (widget.transformationController == null) {
      _transformationController!.dispose();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    Matrix4 transform = _transformationController!.value;
    if (_boundarySizeIsValid) {
      final scale = _transformationController!.value.getMaxScaleOnAxis();
      final centerTranslationOffset = (Offset(
              (_viewport.width - (_boundaryRect.width * scale))
                  .clamp(0, double.infinity),
              (_viewport.height - (_boundaryRect.height * scale))
                  .clamp(0, double.infinity)) /
          2);
      transform = Matrix4.identity()
        ..translate(centerTranslationOffset.dx, centerTranslationOffset.dy)
        ..multiply(transform);
    } else if (_childKey.currentContext == null) {
      WidgetsBinding.instance.addPostFrameCallback((_) => setState(() {}));
    }

    Widget child = ClipRect(
      clipBehavior: widget.clipBehavior,
      child: Transform(
        transform: transform,
        child: Align(
          // This is to ensure the child is correctly aligned when we don't know the size yet
          alignment:
              _boundarySizeIsValid ? Alignment.topLeft : Alignment.center,
          child: KeyedSubtree(
              key: _childKey,
              child: NotificationListener<SizeChangedLayoutNotification>(
                onNotification: (_) {
                  WidgetsBinding.instance
                      .addPostFrameCallback((_) => setState(() {}));
                  return true;
                },
                child: SizeChangedLayoutNotifier(child: widget.child),
              )),
        ),
      ),
    );

    return NotificationListener<SizeChangedLayoutNotification>(
      onNotification: (_) {
        // Obtain new matrix when size changes
        WidgetsBinding.instance.addPostFrameCallback((_) {
          _transformationController!.value = _matrixTranslate(
            _transformationController!.value,
            const Offset(0.000000001, 0.000000001),
          );
          setState(() {});
        });
        return true;
      },
      child: SizeChangedLayoutNotifier(
        child: Listener(
          key: _parentKey,
          onPointerSignal: _receivedPointerSignal,
          child: GestureDetector(
            behavior:
                HitTestBehavior.opaque, // Necessary when panning off screen.
            onScaleEnd: _onScaleEnd,
            onScaleStart: _onScaleStart,
            onScaleUpdate: _onScaleUpdate,
            onDoubleTap: _onDoubleTap,
            onDoubleTapDown: _onDoubleTapDown,
            trackpadScrollCausesScale: widget.trackpadScrollCausesScale,
            trackpadScrollToScaleFactor: Offset(0, -1 / widget.scaleFactor),
            child: child,
          ),
        ),
      ),
    );
  }
}

// A classification of relevant user gestures. Each contiguous user gesture is
// represented by exactly one _GestureType.
enum _GestureType { pan, scale }

// Round the output values. This works around a precision problem where
// values that should have been zero were given as within 10^-10 of zero.
Offset _round(Offset offset) {
  return Offset(
    double.parse(offset.dx.toStringAsFixed(9)),
    double.parse(offset.dy.toStringAsFixed(9)),
  );
}

// Transform the four corners of the viewport by the inverse of the given
// matrix. This gives the viewport after the child has been transformed by the
// given matrix. The viewport transforms as the inverse of the child (i.e.
// moving the child left is equivalent to moving the viewport right).
Quad _transformViewport(Matrix4 matrix, Rect viewport) {
  final Matrix4 inverseMatrix = matrix.clone()..invert();
  return Quad.points(
    inverseMatrix.transform3(Vector3(
      viewport.topLeft.dx,
      viewport.topLeft.dy,
      0.0,
    )),
    inverseMatrix.transform3(Vector3(
      viewport.topRight.dx,
      viewport.topRight.dy,
      0.0,
    )),
    inverseMatrix.transform3(Vector3(
      viewport.bottomRight.dx,
      viewport.bottomRight.dy,
      0.0,
    )),
    inverseMatrix.transform3(Vector3(
      viewport.bottomLeft.dx,
      viewport.bottomLeft.dy,
      0.0,
    )),
  );
}

// Given a velocity and drag, calculate the time at which motion will come to
// a stop, within the margin of effectivelyMotionless.
double _getFinalTime(double velocity, double drag,
    {double effectivelyMotionless = 10}) {
  return math.log(effectivelyMotionless / velocity) / math.log(drag / 100);
}

// Given two points, return the axis where the distance between the points is
// greatest. If they are equal, return null.
Axis? _getPanAxis(Offset point1, Offset point2) {
  if (point1 == point2) {
    return null;
  }
  final double x = point2.dx - point1.dx;
  final double y = point2.dy - point1.dy;
  return x.abs() > y.abs() ? Axis.horizontal : Axis.vertical;
}

// Return the amount that viewport lies outside of boundary. If the viewport
// is completely contained within the boundary (inclusively), then returns
// Offset.zero.
Offset _exceedsBy(Quad boundary, Quad viewport) {
  final List<Vector3> viewportPoints = <Vector3>[
    viewport.point0,
    viewport.point1,
    viewport.point2,
    viewport.point3,
  ];
  Offset largestExcess = Offset.zero;
  for (final Vector3 point in viewportPoints) {
    final Vector3 pointInside =
        MediaViewer.getNearestPointInside(point, boundary);
    final Offset excess = Offset(
      pointInside.x - point.x,
      pointInside.y - point.y,
    );
    if (excess.dx.abs() > largestExcess.dx.abs()) {
      largestExcess = Offset(excess.dx, largestExcess.dy);
    }
    if (excess.dy.abs() > largestExcess.dy.abs()) {
      largestExcess = Offset(largestExcess.dx, excess.dy);
    }
  }

  return _round(largestExcess);
}

// Return the translation from the given Matrix4 as an Offset.
Offset _getMatrixTranslation(Matrix4 matrix) {
  final Vector3 nextTranslation = matrix.getTranslation();
  return Offset(nextTranslation.x, nextTranslation.y);
}

// Find the axis aligned bounding box for the rect rotated about its center by
// the given amount.
Quad _getAxisAlignedBoundingBox(Rect rect) {
  final Quad boundariesRotated = Quad.points(
    Vector3(rect.left, rect.top, 0.0),
    Vector3(rect.right, rect.top, 0.0),
    Vector3(rect.right, rect.bottom, 0.0),
    Vector3(rect.left, rect.bottom, 0.0),
  );
  return MediaViewer.getAxisAlignedBoundingBox(boundariesRotated);
}

// Align the given offset to the given axis by allowing movement only in the
// axis direction.
Offset _alignAxis(Offset offset, Axis axis) {
  switch (axis) {
    case Axis.horizontal:
      return Offset(offset.dx, 0.0);
    case Axis.vertical:
      return Offset(0.0, offset.dy);
  }
}
