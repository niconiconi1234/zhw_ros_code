# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

import maskrcnn_pb2 as maskrcnn__pb2


class MaskRCNNStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.maskrcnn = channel.unary_unary(
                '/MaskRCNN/maskrcnn',
                request_serializer=maskrcnn__pb2.MaskRCNNRequest.SerializeToString,
                response_deserializer=maskrcnn__pb2.MaskRCNNResponse.FromString,
                )


class MaskRCNNServicer(object):
    """Missing associated documentation comment in .proto file."""

    def maskrcnn(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_MaskRCNNServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'maskrcnn': grpc.unary_unary_rpc_method_handler(
                    servicer.maskrcnn,
                    request_deserializer=maskrcnn__pb2.MaskRCNNRequest.FromString,
                    response_serializer=maskrcnn__pb2.MaskRCNNResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'MaskRCNN', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class MaskRCNN(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def maskrcnn(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/MaskRCNN/maskrcnn',
            maskrcnn__pb2.MaskRCNNRequest.SerializeToString,
            maskrcnn__pb2.MaskRCNNResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
